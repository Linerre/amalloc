#include "mymalloc.h"
#include <assert.h>
#include <stdlib.h>
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

/* The smallest possible block with overhead */
#define MIN_BLOCK_SIZE (kMetadataSize + kMinAllocationSize + kAlignment)

/* The smallest aligned size that can be malloced  */
#define MINSIZE        ((MIN_BLOCK_SIZE + ALIGN_MASK) & ~ALIGN_MASK)

/* Round up the requested size to multiple of word size */
#define req2size(req)            (((req) + ALIGN_MASK) & ~ALIGN_MASK)
#define checked_req2size(req)    (((req) + ALIGN_MASK) & ~ALIGN_MASK)

/* ------------------ Heap block operations  ------------------ */
#define CURR_INUSE 0x1
#define PREV_INUSE 0x2
#define SIZE_FLAGS (CURR_INUSE | PREV_INUSE)

#define set_block_size(b, sz) ((b)->size = (sz))

/* Set and unset the flags */
#define set_block_inuse(b)    ((b)->size |= CURR_INUSE)
#define set_block_free(b)     ((b)->size &= ~CURR_INUSE)
#define set_prevb_inuse(b)    ((b)->size |= PREV_INUSE)
#define set_prevb_free(b)     ((b)->size &= ~PREV_INUSE)

#define update_remainder(m, sz)  ((m)->remainder = (sz))

/* Extract payload + overhead  */
#define blocksize(b)         ((b)->size & ~7)

/* Check if `top' would hit the lowest boundary (head of largest bin) */
#define checked_top(m, sz) \
  ((blkptr) (char *)(m)->top - (sz) >= bin_at(m, N_LISTS))

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
  Bins are 1-indexed, so bin 0 does not exit.
 */

/* Bin number */
#define NBINS          (N_LISTS + 1)
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
  N_LISTS)

#define bin_index(sz) \
  ((in_smallbin_range(sz)) ? smallbin_index(sz) : largebin_index(sz))

/* bin_at(0) does not exist */
#define bin_at(m, i) ((m)->bins[i])

/* Reminders about list directionality within bins */
#define first(b)     ((b)->fd)
#define last(b)      ((b)->bk)

#define is_empty(b) (first(b) == (b))
#define has_one(b)  (first(b) == last(b))

/*
  Binmap

  To help compensate for the large number of bins, a one-level index
  structure is used for bin-by-bin searching.  `binmap' is a
  bitvector recording whether bins are definitely empty so they can
  be skipped over during during traversals.  The bits are NOT always
  cleared as soon as bins are empty, but instead only when they are
  noticed to be empty during traversal in malloc.
 */
#define BINMAPSHIFT    6
#define BINMAPSIZE     2
#define BITSPERMAP     32

/* Decide index bit  */
/* #define idx2grp(i)     ((i) >> BINMAPSHIFT) */
#define idx2bit(i)  (1ULL << (((uint64)(i) & ((1ULL << BINMAPSHIFT) - 1))))

#define set_bin(m, i)     ((m)->binmap |= idx2bit(i))
#define unset_bin(m, i)   ((m)->binmap &= ~(idx2bit(i)))
#define get_binmap(m, i)  ((m)->binmap & idx2bit(i))

/* ------------------- Internal state representation ------------------- */

/*
  Global, file-local state for this mallocator implementation.  The state
  tracks information that the allocator should be aware of before some time
  consuming operations (such as coalesing, mmap and splitting) so that it
  can be as lazy as possible.
 */
static struct malloc_state {

  /* Top makrs the end (highest) addr just before the `hfp' below */
  blkptr top;

  /* low fencepost, as start of the 64MB heap */
  blkptr lfp;

  /* high fencepost, as end of the 64MB heap */
  blkptr hfp;

  /* list of bins or size classes */
  blkptr bins[NBINS];

  /* the remaining, allocable space */
  size_t remainder;

  /* 64-bit bitmap for bin indices: effect index starts with 1 */
  uint64 binmap;
} mmstate;

int malloc_initialized = -1;

/*
  Heap memory layout

  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|lfp| bin heads |     free chunks   |   allocated chunks    |tfp|
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  ^                                   ^                       ^
  low fence                          top                 high fence

  At first, all bins are empty except that each bin has a `Block' head
  that points to itself to initialize a circular doudly link list.

  `top' points to the same addr as tfp does but moves towards left as
  allocations happen until there is no space on the left side, that is
  hitting the end of last bin's head.


 */

/* -------------------- Heap and chunk manupilation  ------------------ */
inline static void prepare_block(blkptr blk)
{
  blk->fd = blk->bk = NULL;
  blk = blk + 1;
}

/* See the above diagram for where these two fenceposts are positions */
static void init_fps_top(blkptr init_hh, mstate mst)
{

  size_t rsz = kMemorySize - kMetadataSize;
  blkptr dm_start = init_hh;
  dm_start->size = 1;          /* 1 means initialized */
  dm_start->fd = init_hh + 1;
  dm_start->bk = NULL;
  mst->lfp = dm_start;

  /* TODO: make this a macro */
  blkptr dm_end = (blkptr) ((char*)init_hh + rsz);
  dm_end->size = 1;
  dm_end->fd = NULL;
  dm_end->bk = dm_end - 1;
  mst->hfp = dm_end;

  /* Update top as well */
  mst->top = dm_end->bk;
  rsz -= kMetadataSize;
  set_block_size(mst->top, rsz);
  set_block_free(mst->top);

  update_remainder(mst, rsz);
}

static void init_bins(mstate mst)
{
  int i;
  blkptr bin = mst->lfp + 1;

  /* Establish circular double-linked lists for bins.  These bin heads
     stored at heap start, immediately after the top fencepost */
  for (i = 1; i < NBINS; ++i) {
    mst->bins[i] = bin;
    bin->fd = bin->bk = bin;
    bin++;
  }

  size_t rsz = mst->remainder - N_LISTS * kMetadataSize;
  set_block_size(mst->top, rsz);
  update_remainder(mst, rsz);
}

/*
  Initialize the `heap', which is the memory chunk totaling 64MB
  to be used for future allocations unless it can no longer satisfiy
 */
static blkptr init_heap(void)
{
  blkptr heap = mmap(NULL,                        /* let kernel decide */
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


  mstate mst = &mmstate;

  /* Set fenceposts at both start and end of the heap */
  init_fps_top(heap, mst);

  /* Init bins and place the initial chunk in the largest bin */
  init_bins(mst);

  /* Init bin map */
  mst->binmap = 0;

  /* Mark initialization */
  malloc_initialized = 0;

  return mst->top;
}

/* Start with current empty bin and search for the nearest next
   non-empty larger bin.  Return its index or -1 for not found. */
int find_next_nonempty_bin(mstate m, int bidx) {

  uint64 map = m->binmap;

  /* Clear all bits lower than the idx */
  map &= ~(idx2bit(bidx) - 1);

  if (map == 0) {
    LOG("Bin at index %d does not exit\n", idx);
    return -1;
  }

  /* Find the first set bit in the non-zero map */
  while ((map & 1) == 0) {
    map >>= 1;
    bidx++;
  }

  return bidx;
}

/* Insert a free block immediately after the head of a suitable bin */
void insert_into_bin(mstate m, blkptr blk) {
  size_t size = blocksize(blk);
  int bidx = bin_index(size);
  blkptr bin = bin_at(m, bidx);

  /* Insert at the head of the bin list */
  if (is_empty(bin)) {
    blk->fd = bin;
    blk->bk = bin;
    bin->fd = blk;
    bin->bk = blk;
  } else {
    blk->fd = bin->fd;
    blk->bk = bin;
    bin->fd->bk = blk;
    bin->fd = blk;
  }

  /* FIXME: avoid repeated marking? */
  /* Mark the bin as non-empty in the binmap */
  set_bin(m, bidx);
}

/* Split the `blk' at its high addr end for allocation, user getting
   the high-addr part, remaining low-addr part as free */
static blkptr split_block(blkptr blk, size_t asize)
{
  size_t rs = blocksize(blk) - asize - kMetadataSize;
  blkptr mblock;

  if (rs >= MINSIZE) {          /* remaining allocable on its own */
    /* FIXME: add footer? */
    mblock = (blkptr) ((char*) blk + rs);
    set_block_size(mblock, asize + kMetadataSize);
    set_block_inuse(mblock);
    set_block_size(blk, rs);
    return blk;
  } else {                      /* remaining allocated as well */
    mblock = blk;
    set_block_size(mblock, mblock->size);
    set_block_inuse(mblock);
    return NULL;
  }
}

/* Allocate a free block from given non-empty bin, taking the last
   free block in the bin and returning pointer to the usable addr of
   the free block.   */
static blkptr __alloc_from_bin(mstate m, int bidx)
{
  blkptr bin = bin_at(m, bidx);
  blkptr fblk;

  if (has_one(bin)) {           /* only one free block */
    fblk = bin->fd;
    /* unlink */
    bin->fd = bin->bk = bin;
  } else {                      /* 2 or more free blocks */
    fblk = bin->bk;
    /* unlink */
    bin->bk = bin->bk->bk;
    bin->bk->fd = bin;
  }

  set_block_inuse(fblk);

  return fblk;
}

static blkptr __alloc_from_top(mstate m, size_t sz)
{
  blkptr mblk = m->top;
  mblk = (blkptr) ((char *)mblk - sz + kMetadataSize);
  set_block_size(mblk, sz);
  set_block_inuse(mblk);
  /* TODO: zero the free block? */

  m->top = mblk - 1;
  update_remainder(m, m->remainder - sz);

  return mblk;
}


/* ----------------- Required helper functions ----------------- */
/** These are helper functions you are required to implement for
 *  internal testing purposes. Depending on the optimisations you
 *  implement, you will need to update these functions yourself.
 **/

/* Returns 1 if the given block is free, 0 if not. */
int is_free(blkptr block) {
  return ~(block->size & 1);
}

/* Returns the size of the given block */
size_t block_size(blkptr block) {
  return blocksize(block);
}

/* Returns the first block in memory (excluding fenceposts) */
blkptr get_start_block(void) {
  return NULL;
}

/* Returns the next block in memory */
blkptr get_next_block(blkptr block) {
  return NULL;
}

/* Given a ptr assumed to be returned from a previous call to `my_malloc`,
   return a pointer to the start of the metadata block. */
blkptr ptr_to_block(void *ptr) {
  return ADD_BYTES(ptr, -(kMetadataSize));
}

/* ----------------- Core malloc/free functions ----------------- */
/*
  `size' is user requested when calling this allocator. For a single
  call of this function each time, word_size <= size <= 128MB;
 */
void* my_malloc(size_t size)
{
  /* FIXME: handle size >= kMaxAllocationSize */
  if (size < kMinAllocationSize)
    return NULL;

  blkptr heap;
  mstate mst = &mmstate;

  if (malloc_initialized < 0) {
    heap = init_heap();

    if (heap == NULL)
      return NULL;
  }
  else
    heap = mst->top;


  /* 1. align requested size to multiple of 8 */
  size_t blk_asize = req2size(size);
  size_t total_asz = blk_asize + kMetadataSize;

  /* 2. determin size class list */
  int bidx = bin_index(blk_asize);

  /*
    Searching for a suitable block
    if match-bin-non-empty
      allocate from bin
    else if top has space
      allocate from top
    else
      search through larger bins
      try coalecing first
   */

  if (get_binmap(mst, bidx)) {
    heap = __alloc_from_bin(mst, bidx);
  }
  else if (checked_top(mst, total_asz)) {
    heap = __alloc_from_top(mst, total_asz);
  }
  else {
    /* search for the next larger non-empty first */
    int nbidx = find_next_nonempty_bin(mst, bidx);
    if (nbidx > 0) {            /* splitting */
      blkptr rm = split_block(heap, blk_asize);
      if (rm != NULL)
        insert_into_bin(mst, rm);
    }
    else {                      /* last hope before new mmap syscall */
      /* TODO: coalescing if no bin satiscify and try to allocate */
    }

  }

  /* 4. allocate */
  prepare_block(heap);
  return heap;

}

void my_free(void *ptr) {
  return;
}

/* ----------------- Test main ----------------- */
int main(int argc, char* argv[])
{

  int* fib = my_malloc(173); /* aligned to 176 */

  printf("Allocated heap start = %p\n", fib);


  int i;
  fib[0] = 0;
  fib[1] = 1;

  for (i=2;i<10;i++)
    fib[i] = fib[i-1] + fib[i-2];


  /* assert (bptr != NULL); */
  /* test small bin index */
  /* int is_small = in_smallbin_range(128); */
  /* int idx = bin_index(req2size(129)); */

  /* printf("is small? %d\n", is_small); */
  return 0;
}
