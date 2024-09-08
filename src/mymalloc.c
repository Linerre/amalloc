#include "mymalloc.h"

/* --------------------- Assigment requirements --------------------- */
const size_t kAlignment = sizeof(size_t);
/* Minimum allocation size (1 word) */
const size_t kMinAllocationSize = kAlignment;
/* Size of meta-data per Block */
const size_t kMetadataSize = sizeof(Block);
/* Maximum allocation size (128 MB) */
const size_t kMaxAllocationSize = (128ull << 20) - kMetadataSize;
/* Memory size that is mmapped (64 MB) */
const size_t kMemorySize = (64ull << 20);

/* ------------------ Size and alignment checks ------------------ */
#define ALIGN_MASK     (kAlignment - 1)

/* The smallest possible chunk with header object and size info at end */
#define MIN_CHUNK_SIZE (kMetadataSize + kMinAllocationSize + kAlignment)

/* The smallest aligned size that can be malloced  */
#define MINSIZE        ((MIN_CHUNK_SIZE + ALIGN_MASK) & ~ALIGN_MASK)

/* Round up the requested size to multiple of word size */
#define req2size(req)    (((req) + ALIGN_MASK) & ~ALIGN_MASK)


/* --------------------- Internal data structures --------------------- */
/*
  Segerated lists (Bins) with each list aligned to multiples of 8 bytes.

  Indexing

  List for size <= 256 bytes contain chunks of all the same size, spaced
  8 bytes apart.  Larger lists are approximately logarithmically spaced:

  32 bins of size            8   FIFO
  16 bins of size           64   FIFO
   8 bins of size          512   FIFO
   2 bins of size         4096   FIFO
   1 bins of size  what's left   chunks kept in size order

  Macros for computing bin indices assume arg `sz' to be of type `size_t'.
 */

/* Small bin upper bound */
static const size_t SBINBD = 8UL * 32UL;
/* Medium bin upper bound */
static const size_t MBINBD = SBINBD + 16UL * 64UL;
/* Large bin upper bound  */
static const size_t LBINBD = MBINBD + 4UL * 512UL;
/* Huge bin upper bound */
static const size_t HBINBD = LBINBD + 2UL * 4096UL;
/* 32 small bins */
#define NSMALLBINS     32
/* Spaced 8 bytes apart  */
#define SMALLBIN_SPACE 8
/* Handles size ranging from 8 (min) to 256 (max) */
#define MAX_SMALL_SIZE (NSMALLBINS * SMALLBIN_SPACE)

#define in_smallbin_range(sz) \
  ((sz) < (size_t) MAX_SMALL_SIZE)

#define smallbin_index(sz) ((sz) >> 3)

#define largebin_index(sz) \
  (((((sz) - MAX_SMALL_SIZE) >> 6) < 16) ? 32 + (((sz) - MAX_SMALL_SIZE) >> 6) :\
   ((((sz) - MAX_SMALL_SIZE) >> 9) < 8)  ? 48 + (((sz) - MAX_SMALL_SIZE) >> 9) :\
   ((((sz) - MAX_SMALL_SIZE) >> 12) < 2) ? 56 + (((sz) - MAX_SMALL_SIZE) >> 12) :\
  N_LISTS - 1)

#define bin_index(sz) \
  ((in_smallbin_range(sz)) ? smallbin_index(sz) : largebin_index(sz))

/*
  Binmap

  To help compensate for the large number of bins, a one-level index
  structure is used for bin-by-bin searching.  `binmap' is a
  bitvector recording whether bins are definitely empty so they can
  be skipped over during during traversals.  The bits are NOT always
  cleared as soon as bins are empty, but instead only when they are
  noticed to be empty during traversal in malloc.
 */
#define BINMAPSHIFT    5
#define BINMAPSIZE     2
#define BITSPERMAP     32

/* Decide index group and bit  */
#define idx2grp(i)     ((i) >> BINMAPSHIFT)
#define idx2bit(i)     (1U << (((i) & ((1U << BINMAPSHIFT) - 1))))

#define set_bin(m, i)      ((m)->binmap[idx2grp(i)] |= idx2bit(i))
#define unset_bin(m, i)    ((m)->binmap[idx2grp(i)] &= ~(idx2bit(i)))
#define check_binmap(m, i) ((m)->binmap[idx2grp(i)] & idx2bit(i))


/* ------------------- Internal state representation ------------------- */

/*
  Global, file-local state for this mallocator implementation.  The state
  tracks information that the allocator should be aware of before some time
  consuming operations (such as coalesing, mmap and splitting) so that it
  can be as lazy as possible.
 */
static struct malloc_state {
  /* list of bins or size classes */
  Block *bins[N_LISTS];

  /* Top just after the `ftp' below */
  Block *top;

  /* Top fencepost, as start of the 64MB heap; grows to higher addr */
  Block *tfp;

  /* Bot fencepost, as end of the 64MB heap and at very higher addr */
  Block *bfp;

  /* the remaining, allocable space */
  size_t remainder;

  unsigned int binmap[BINMAPSIZE];
} mmstate;

/* -------------------- Heap and chunk manupilation  ------------------ */

/* Check if heap initialized: uninit heap unmarked tfp and bfp */
#define need_init(m)        (m->tfp->header == 0)

/* Set and unset the allocated bit of header */
#define set_abit(bptr)   ((bptr)->header |= 1)
#define unset_abit(bptr) ((bptr)->header &= (~1))

/* Get chunk size from header field without modifying use bits */
/* FIXME: make it work with prev inuse bit as well */
#define chunksize(bptr) ((bptr)->header & ~2)

void set_pbit(Block* bptr)
{
  bptr->header |= (1 << 1);
}

void unset_pbit(Block* bptr)
{
  bptr->header &= ~(1 << 1);
}


/* Helper functions for seglists */

size_t find_sclass(size_t asize) /* aligned size */
{
  return 0;
}

inline static void prepare_chunk(Block *chunk)
{
  chunk->next = chunk->prev = NULL;
}

static void init_fps(Block *init_heaphead, mstate mst)
{

  Block *dm_head = init_heaphead;
  dm_head->header = 1;          /* 1 means initialized */
  dm_head->next = init_heaphead + 1;
  dm_head->prev = NULL;
  mst->tfp = dm_head;

  /* Set top to be the addr just after `dm_head' for very 1st alloc */
  mst->top = dm_head->next;

  Block *dm_tail = (Block*) ((char*)init_heaphead + (kMemorySize - kMetadataSize));
  dm_tail->header = 1;
  dm_tail->next = NULL;
  dm_tail->prev = dm_tail - 1;
  mst->bfp = dm_tail;

  mst->remainder = kMemorySize - (2 * kMetadataSize);
}

static void init_bins(mstate mst)
{
  int i;
  Block *bin = mst->top;

  /* Establish circular double-linked lists for bins.  These bin heads
     stored at heap start, immediately after the top fencepost */
  for (i = 0; i < N_LISTS; ++i) {
    mst->bins[i] = bin;
    bin->next = bin->prev = bin;
    bin++;
  }

  mst->remainder -= N_LISTS * kMetadataSize;
  mst->top = bin;
  mst->top->header = mst->remainder;
}
/*
  Initialize the `heap', which is the memory chunk totaling 64MB
  to be used for future allocations unless it can no longer satisfiy
 */
static Block* init_heap(void)
{
  Block* heap = mmap(NULL,                        /* let kernel decide */
                     kMemorySize,                 /* 64MB at a time    */
                     PROT_READ | PROT_WRITE,      /* prot              */
                     MAP_PRIVATE | MAP_ANONYMOUS, /* flags             */
                     0,                           /* fd                */
                     0                            /* offset            */
                     );

  if (heap == MAP_FAILED) {
    LOG("mmap failed: %s\n", strerror(errno));
    return NULL;
  }

  /* Set fenceposts at both start and end of the heap */
  mstate mst = &mmstate;
  init_fps(heap, mst);

  /* Init bins and place the initial chunk in the largest bin */
  init_bins(mst);

  /* Init bin map */
  mst->binmap[0] = 0;
  mst->binmap[1] = 1U << 26;

  return mst->top;
}

/* ----------------- Required helper functions ----------------- */
/** These are helper functions you are required to implement for
 *  internal testing purposes. Depending on the optimisations you
 *  implement, you will need to update these functions yourself.
 **/

/* Returns 1 if the given block is free, 0 if not. */
int is_free(Block *block) {
  return ~(block->header & 1);
}

/* Returns the size of the given block */
size_t block_size(Block *block) {
  return chunksize(block);
}

/* Returns the first block in memory (excluding fenceposts) */
Block *get_start_block(void) {
  return NULL;
}

/* Returns the next block in memory */
Block *get_next_block(Block *block) {
  return NULL;
}

/* Given a ptr assumed to be returned from a previous call to `my_malloc`,
   return a pointer to the start of the metadata block. */
Block *ptr_to_block(void *ptr) {
  return ADD_BYTES(ptr, -(kMetadataSize));
}

/* ----------------- Core malloc/free functions ----------------- */
/*
  `size' is user requested when calling this allocator. For a single
  call of this function each time, word_size <= size <= 128MB;
 */
void *my_malloc(size_t size) {
  /* FIXME: handle size >= kMaxAllocationSize */
  if (size < kMinAllocationSize)
    return NULL;

  Block *heap;
  struct malloc_state *mst = &mmstate;

  if (need_init(mst)) {
    heap = (Block*) init_heap();

    if (heap == NULL)
      return NULL;
  }
  else
    heap = mst->top;


  /* 1. align requested size to multiple of 8 */
  size_t chunk_size = req2size(size);

  /* 2. determin size class list */
  int bindex = bin_index(chunk_size);

  /* 3. find a best-fit free block for allocation */
  if (check_binmap(mst, bindex)) {
    Block *bin = mst->bins[bindex];
    if (bin->next == bin) { /* only 1 free block in this bin */
      heap = bin;
      prepare_chunk(heap);
      /* update binmap */
    } else if (bin->next == bin->prev) { /* two free blocks */
      heap = bin->next;
      prepare_chunk(heap);
      /* unlink */
      bin->next = bin->prev = bin;
    } else {                    /* three or more free blocks */
      heap = bin->prev;
      /* unlink */
      bin->prev = bin->prev->prev;
      bin->prev->next = bin;
      prepare_chunk(heap);
    }

  } else {
    int grp = idx2grp(bindex);
    unsigned int bit = idx2bit(bindex);
    unsigned int map = mst->binmap[grp];

    // Clear all bits before the start_idx in this block
    map &= ~(bit - 1);

    // Find the next set bit in the current group or next group
    while (map == 0) {
        // Move to the next group
        grp++;
        if (grp >= BINMAPSIZE) {
          // We've reached the end of the binmap
          LOG("Failed to find a bin");
          return NULL;
        }
        map = mst->binmap[grp];
        bindex = grp * BITSPERMAP;
    }

    // Find the first set bit in the non-zero map
    while ((map & 1) == 0) {
        map >>= 1;
        bindex++;
    }


    Block *bin = mst->bins[bindex];
    if (bin->next == bin) { /* only 1 free block in this bin */
      heap = bin;
      prepare_chunk(heap);
      /* update binmap */
    } else if (bin->next == bin->prev) { /* two free blocks */
      heap = bin->next;
      prepare_chunk(heap);
      /* unlink */
      bin->next = bin->prev = bin;
    } else {                    /* three or more free blocks */
      heap = bin->prev;
      /* unlink */
      bin->prev = bin->prev->prev;
      bin->prev->next = bin;
      prepare_chunk(heap);
    }
  }

  /* 4. allocate */
  return heap;

}

void my_free(void *ptr) {
  return;
}

/* ----------------- Test main ----------------- */
int main(int argc, char* argv[])
{

  Block* bptr = init_heap();

  /* test small bin index */
  int is_small = in_smallbin_range(128);
  int idx = bin_index(req2size(129));

  printf("is small? %d\n", is_small);
  return 0;
}
