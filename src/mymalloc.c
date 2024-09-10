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
#define TAGS_SIZE      (kMetadataSize - kAlignment)

/* The smallest possible block with overhead */
#define MIN_BLOCK_SIZE (kMetadataSize + kMinAllocationSize + kAlignment)

/* The smallest aligned size that can be malloced  */
#define MINSIZE        ((MIN_BLOCK_SIZE + ALIGN_MASK) & ~ALIGN_MASK)

/* Round up the requested size to multiple of word size */
#define req2size(req)          (((req) + ALIGN_MASK) & ~ALIGN_MASK)
/* #define checked_req2size(req)  (((req) + ALIGN_MASK) & ~ALIGN_MASK) */

/* ------------------ Heap block operations  ------------------ */
#define CURR_INUSE 0x1
#define PREV_INUSE 0x2
#define SIZE_FLAGS (CURR_INUSE | PREV_INUSE)

/* Extract payload + overhead  */
#define blocksize(b)         ((b)->size & ~7)

#define set_block_size(b, sz) ((b)->size = (sz))
#define set_block_foot(b) \
{ \
  size_t sz = blocksize(b); \
  size_t *ft = (size_t *) ((char *)b + sz - kAlignment); \
  *ft = sz; \
}

/* Set and unset the flags */
#define set_block_inuse(b)    ((b)->size |= CURR_INUSE)
#define set_block_free(b)     ((b)->size &= ~CURR_INUSE)
#define set_prevb_inuse(b)    ((b)->size |= PREV_INUSE)
#define set_prevb_free(b)     ((b)->size &= ~PREV_INUSE)

#define update_remainder(m, sz)  ((m)->remainder = (sz))
#define prepare_allocblk(b) \
  ((b) = (blkptr) ((char *)b - (2 * sizeof(blkptr))))

#define update_top(m, b) ((m)->top = (b) - 1);

#define at_lfp(m)    ((char *) ((m)->lfp) + kMetadataSize)
#define at_hfp(m)    ((char *) ((m)->hfp))

/* Check if `top' would hit the lowest boundary (lfp) */
#define checked_top(m, sz) \
  (((char *)((m)->top) - (sz)) >= at_lfp(m))

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

/* #define is_empty(b) (first(b) == (b)) */
#define is_empty(b)  ((b) == NULL)
#define has_one(b)   ((b) != NULL && first(b) == (b))
#define has_two(b)   ((b) != NULL && first(b) == last(b))
#define not_at_bintail(blk, bin)  ((blk)->fd != (bin))
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
#define idx2bit(i)  (1ULL << (((uint64)(i) & ((1ULL << BINMAPSHIFT) - 1))))

#define set_binmap(m, i)    ((m)->binmap |= idx2bit(i))
#define unset_binmap(m, i)  ((m)->binmap &= ~(idx2bit(i)))
#define get_binmap(m, i)    ((m)->binmap & idx2bit(i))

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
/* See the above diagram for positions of these two fenceposts  */
static void init_fps_top(blkptr init_hh, mstate ms)
{

  size_t rsz = kMemorySize - kMetadataSize;
  blkptr dm_start = init_hh;
  dm_start->size = 1;          /* 1 means initialized */
  dm_start->fd = init_hh + 1;
  dm_start->bk = NULL;
  ms->lfp = dm_start;

  /* TODO: make this a macro */
  blkptr dm_end = (blkptr) ((char*)init_hh + rsz);
  dm_end->size = 1;
  dm_end->fd = NULL;
  dm_end->bk = dm_end - 1;
  ms->hfp = dm_end;

  /* Update top */
  ms->top = dm_end->bk;
  rsz -= kMetadataSize;
  set_block_size(ms->top, rsz);
  set_block_free(ms->top);

  update_remainder(ms, rsz);
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


  mstate ms = &mmstate;

  /* Set fenceposts at both start and end of the heap */
  init_fps_top(heap, ms);

  /* Init bins and place the initial chunk in the largest bin */
  /* init_bins(ms); */

  /* Init bin map */
  ms->binmap = 0;

  /* Mark initialization */
  malloc_initialized = 0;

  return ms->top;
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

/* Insert a free block at head of the given bin */
void insert_into_bin(int bidx, blkptr fblk) {

  mstate m = &mmstate;

  blkptr bin = bin_at(m, bidx);

  if (is_empty(bin)) {
    fblk->fd = bin;
    fblk->bk = bin;
    bin->fd = fblk;
    bin->bk = fblk;
    set_binmap(m, bidx);
  } else {
    fblk->fd = bin->fd;
    fblk->bk = bin;
    bin->fd->bk = fblk;
    bin->fd = fblk;
  }
}

/* Split the larger `blk' at higher addr end and return the rightmost
   part for malloc, putting remaining left (low-addr) part back to a
   suitable bin. */
static blkptr split_block(blkptr blk, size_t asize)
{
  size_t nsz = asize + kMetadataSize; /* needed size */
  size_t rsz = blocksize(blk) - nsz;  /* remaining size */
  blkptr lblk = blk;
  blkptr rblk;

  if (rsz >= MINSIZE) {         /* remainder allocable on its own */
    /* take the right block */
    rblk = (blkptr) ((char*) blk + rsz);
    set_block_size(rblk, nsz);
    set_block_inuse(rblk);
    set_block_foot(rblk)
    prepare_allocblk(rblk);

    /* collect the left block */
    set_block_size(lblk, rsz);
    insert_into_bin(bin_index(rsz), lblk);

    return rblk;

  } else {                      /* remainder allocated as well */
    set_block_size(lblk, lblk->size);
    set_block_inuse(lblk);
    set_block_foot(lblk)
    prepare_allocblk(lblk);

    return lblk;
  }
}

/* Allocate a free block from given non-empty bin, taking the last
   free block in the bin and returning pointer to the usable addr of
   the free block.   */
static blkptr __alloc_from_bin(mstate m, int bidx)
{
  blkptr bin = bin_at(m, bidx);

  assert(bin != NULL);

  blkptr mblk;

  if (has_one(bin)) {           /* only one free block */
    mblk = bin;
    /* unlink */
    bin = NULL;
  } else if (has_two(bin)) {    /* 2 free blocks */
    mblk = bin->bk;
    /* unlink */
    bin->fd = bin->bk = bin;
  } else {                      /* 3 or more free blocks */
    mblk = bin->bk;
    /* unlink */
    bin->bk = bin->bk->bk;
    bin->bk->fd = bin;
  }

  set_block_inuse(mblk);
  set_block_foot(mblk)

  return mblk;
}

/* `tsz` is total size for the to-be-allocated block, including both
   the block size returned to user and overhead (boundary tags)   */
static blkptr __alloc_from_top(mstate m, size_t tsz)
{
  blkptr mblk = m->top;

  mblk = (blkptr) ((char *)mblk - tsz);

  set_block_size(mblk, tsz);
  set_block_inuse(mblk);

  set_block_foot(mblk)

  prepare_allocblk(mblk);

  update_top(m, mblk);
  update_remainder(m, m->remainder - tsz);

  return mblk;
}

/* Try to find a larger free block from the bins after the current
   `idx' for malloc.  The found block will be split first (see
   `split_block').  Return the malloc block on success and NULL on
   failure. */
static blkptr __try_alloc_from_next(mstate m, int bidx, size_t basize)
{
  int nbidx = find_next_nonempty_bin(m, bidx);

  if (nbidx > 0) {
    blkptr mblk = bin_at(m, nbidx);
    return split_block(mblk, basize);
  }

  return NULL;
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
  mstate ms = &mmstate;

  if (malloc_initialized < 0) {
    heap = init_heap();

    if (heap == NULL)
      return NULL;
  }
  else
    heap = ms->top;


  /* 1. align requested size to multiple of 8 */
  size_t basize = req2size(size);
  size_t total_asz = basize + TAGS_SIZE;

  /* 2. determin size class list */
  int bidx = bin_index(basize);

  /*
    Searching for a suitable block
    if match-bin-non-empty
      allocate from bin
    else if top has space
      allocate from top
    else
      1. try search next larger bins and if failed, then
      2. try coalecing then and if failed, then
      3. request new huge chunck via mmap
   */

  if (get_binmap(ms, bidx)) {
    return __alloc_from_bin(ms, bidx);
  }
  else if (checked_top(ms, total_asz)) {
    return __alloc_from_top(ms, total_asz);
  }
  else {
    /* try to alloc from the next larger non-empty  */
    heap =  __try_alloc_from_next(ms, bidx, basize);

    /* last hope before new mmap: coalescing */
    if (heap == NULL) {
      /* TODO: coalesce  */
    }
  }

  /* 4. allocate */
  return heap;

}

void my_free(void *ptr) {

  /*
    1. cast ptr to blkptr
    2. update tags/flags properly
    3. use size to get bin index
    4. put blk back to corresponding bin
       if bin index == N_LISTS
         chunks ordered in size
       else
         insert at head

    Coalescing when freeing

    if small bin block, insert into suitable bin
    if medium or larger, try coalescing first
      if succeed, insert into suitable bin
      else quit coalescing until next time

   */

  mstate m = &mmstate;
  blkptr mblk = (blkptr) ptr;
  set_block_free(mblk);

  size_t blksz = blocksize(mblk);
  int bidx = bin_index(blksz);


  blkptr bin = m->bins[bidx];
  if (bidx == N_LISTS) {
    if (is_empty(bin)) {
      bin = mblk;
      bin->fd = bin->bk = mblk;
      return;
    } else if (has_one(bin)) {  /* 1 free blocks only */
      mblk->fd = bin;
      mblk->bk = bin;
      bin->fd = mblk;
      bin->bk = mblk;

      /* update bin head if mblk is the smallest */
      if (blksz <= blocksize(bin))
        bin = mblk;

      return;
    } else {                    /* 2 or more free blocks */
      blkptr curr = bin;
      while(not_at_bintail(curr, bin)) {
        /* keep moving if mblk larger than curr */
        if (blksz >= blocksize(curr)) {
          curr = curr->fd;
          continue;
        }

        /* else put mlbk before curr */
        mblk->fd = curr;
        mblk->bk = curr->bk;
        curr->bk->fd = mblk;
        curr->bk = mblk;

        if (curr == bin)
          bin = mblk;

        return;
      }

      /* at bin tail and mblk is the largest */
      mblk->fd = curr->fd;
      mblk->bk = curr;
      curr->fd = mblk;
      bin->bk = mblk;

      return;
    }                           /* end in last largest bin */

  } else {                      /* in one of the previous bins  */
    if (is_empty(bin)){
      bin = mblk;
      bin->fd = bin->bk = mblk;
    }
    else if (has_one(bin)) {    /* 1 free blocks only */
      mblk->fd = bin;
      mblk->bk = bin;
      bin->fd = mblk;
      bin->bk = mblk;

      bin = mblk;
    } else {                    /* 2 or more free blocks */
      mblk->fd = bin;
      mblk->bk = bin->bk;
      bin->bk->fd = mblk;       /* tail points to mblk */
      bin->bk = mblk;           /* orig head points to mblk */
      bin = mblk;               /* mblk is head now  */
    }

    return;
  }

}

/* ----------------- Test main ----------------- */
int main(int argc, char* argv[])
{

  int* fib = my_malloc(173); /* aligned to 192 (3 padding + 16 boundary tags) */

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
