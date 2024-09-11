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
#define SIZE_FLAGS (CURR_INUSE | PREV_INUSE)

/* size field is OR'ed with this flag when previous adjacent block in use */
#define PREV_INUSE 0x1
#define FTR_SZ     ((size_t) 8)
#define HDR_SZ     ((size_t) 8)

/* Get block size (payload + overhead), ignoring PREV_INUSE bit */
#define blocksize(b)         ((b)->size & ~ALIGN_MASK)

/* Like blocksize, but contains the PREV_INUSE bit */
#define blocksize_nomask(b)  ((b)->size)

#define set_size(b, sz) ((b)->size = (sz))

/* Set and unset the flags */
#define set_block_inuse(b)    ((b)->size |= CURR_INUSE)
#define set_block_free(b)     ((b)->size &= ~CURR_INUSE)
#define set_prevb_inuse(b)    ((b)->size |= PREV_INUSE)
#define set_prevb_free(b)     ((b)->size &= ~PREV_INUSE)

/* Set size at head, without disturbing its use bit */
#define set_head_size(b, s)   ((b)->size = (((b)->size & PREV_INUSE) | (s)))

/* Set size/use field at the same time */
#define set_head(b, s)  ((b)->size = (s))

/* Set size based on off for footer (used only for free blocks) */
#define set_foot(b, off, sz) \
  (((blkptr) ((char *) (b) + ((off) - FTR_SZ)))->size = (sz))

/* Treat space at ptr + offset as a block */
#define block_at_offset(b, s)  ((blkptr) (((char *) (b)) + (s)))

/* extract b's inuse bit */
#define inuse(b) \
  ((((blkptr) (((char *) (b)) + blocksize(b)))->size) & PREV_INUSE)

/* set/clear block as being inuse without otherwise disturbing */
#define set_inuse(b) \
  ((blkptr) (((char *) (b)) + blocksize(b)))->size |= PREV_INUSE

#define unset_inuse(b) \
  ((blkptr) (((char *) (b)) + blocksize(b)))->size &= ~(PREV_INUSE)

/* Move block metadata pointer to the start of usable memory */
#define set_block_memp(b) \
  ((b) = (blkptr) ((char *)b + kAlignment))

/* Move usable memory pointer to the start of block metadata */
#define unset_block_memp(b) \
  ((b) = (blkptr) ((char *)b - kAlignment))

#define inc_remainder(m, sz) ((m)->remainder += (sz))
#define dec_remainder(m, sz) ((m)->remainder -= (sz))
#define set_remainder(m, sz) ((m)->remainder = (sz))

/* Move top forward (to higher addr) or backward (to lower addr) */
#define fd_top(m, b, sz)  ((m)->top = (blkptr)((char *)(b) + sz))
#define bk_top(m, b, sz)  ((m)->top = (blkptr)((char *)(b) - sz))

#define at_lfp(m)    ((char *) ((m)->lfp) + kMetadataSize)
#define at_hfp(m)    ((char *) ((m)->hfp))

/* Check if `top' would hit the highest boundary (hfp) */
#define checked_top(m, sz) \
  (((char *)((m)->top) + (sz)) <= at_hfp(m))

/* --------------------- Internal data structures --------------------- */
/*
  Segerated lists (Bins) with each list aligned to multiples of 8 bytes.

  Indexing

  List for size <= 256 bytes contain chunks of all the same size, spaced
  8 bytes apart.  Larger lists are approximately logarithmically spaced:

  -----------------------------------------------------------------
  lists in bin          spaced   Order                 Range (byte)
  -----------------------------------------------------------------
  32 bins of size            8   FIFO                    8 -   256
  16 bins of size           64   FIFO                  257 -  1280
   8 bins of size          512   FIFO                 1281 -  5376
   2 bins of size         4096   FIFO                 5377 - 13568
   1 bins of size  what's left   kept in size order  13568+
  -----------------------------------------------------------------

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
#define bin_at(m, i)           ((m)->bins[i])
#define init_bin(m, i, b)      ((m)->bins[i] = (b))

/* set head of bin at i to b*/
#define set_bin_head(m, i, b)  ((m)->bins[i] = b)
/* set bin head to NULL */
#define nil_bin_head(m, i)     ((m)->bins[i] = NULL)

/* Reminders about list directionality within bins */
#define first(b)     ((b)->fd)
#define last(b)      ((b)->bk)

/* #define is_empty(b) (first(b) == (b)) */
#define is_empty(b)  ((b) == NULL)
#define has_one(b)   ((b) != NULL && first(b) == (b))
#define has_two(b)   ((b) != NULL && first(b) != (b) && first(b) == last(b))
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
	|lfp| allocated chunks  |   free chunks                     |tfp|
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  ^                       ^                                   ^
  low fence               top                             high fence

  At first, all bins are empty except that each bin has a `Block' head
  that points to itself to initialize a circular doudly link list.

  `top' points to the addr immediately after lfp at init stage and moves
  towards right (to higher addr) while allocating until it hits hfp.


 */

/* -------------------- Heap and chunk manupilation  ------------------ */
/* See the above diagram for positions of these two fenceposts  */
static void init_fps_top(blkptr init_hh, mstate ms)
{

  size_t rsz = kMemorySize - kMetadataSize;
  blkptr dm_start = init_hh;
  dm_start->size = 1;          /* memory beyond leftmost alwasy in use */
  dm_start->fd = init_hh + 1;
  dm_start->bk = NULL;
  ms->lfp = dm_start;

  /* TODO: make this a macro */
  blkptr dm_end = (blkptr) ((char*)init_hh + rsz);
  dm_end->size = 0;             /* memory < rightmost always free at start */
  dm_end->fd = NULL;
  dm_end->bk = dm_end - 1;
  ms->hfp = dm_end;

  /* Update top */
  ms->top = dm_start + 1;
  rsz -= kMetadataSize;
  set_size(ms->top, rsz);
  set_block_free(ms->top);

  set_remainder(ms, rsz);
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

/* Normal bins are those that belong to bins 1-58 (i.e. not in the
   last bin).  These bins are circular doubly linked lists that act
   in a FIFO mannar: inserting at heap and taking from tail. */
static void __insert_normal_bins(int bidx, blkptr mblk)
{
  mstate m = &mmstate;
  blkptr bin = bin_at(m, bidx);

  if (is_empty(bin)){
    init_bin(m, bidx, mblk);
    bin = mblk;
    bin->fd = bin->bk = mblk;
    set_binmap(m, bidx);
  } else if (has_one(bin)) {    /* 1 free blocks only */
    mblk->fd = bin;
    mblk->bk = bin;
    bin->fd = mblk;
    bin->bk = mblk;
    bin = mblk;
  } else {                      /* 2 or more free blocks */
    mblk->fd = bin;
    mblk->bk = bin->bk;
    bin->bk->fd = mblk;         /* tail points to mblk */
    bin->bk = mblk;             /* orig head points to mblk */
    bin = mblk;                 /* mblk is head now  */
  }
}

/* Blocks in the last bins are arranged according their sizes, from
   small to large, to meet the best-fit requirement. */
static void __insert_last_bin(blkptr mblk)
{
  mstate m = &mmstate;
  blkptr bin = bin_at(m, N_LISTS);
  size_t blksz = blocksize(mblk);

  if (is_empty(bin)) {
    init_bin(m, N_LISTS, mblk);
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
  }
}

/* Insert a free block at head of the given bin */
static void insert_into_bin(int bidx, blkptr mblk) {

  if (bidx == N_LISTS)
    __insert_last_bin(mblk);
  else
    __insert_normal_bins(bidx, mblk);
}

/* Split the larger `blk' at higher addr end and return the rightmost
   part for malloc, putting remaining left (low-addr) part back to a
   suitable bin. */
static blkptr split_block(blkptr blk, size_t asize)
{
  size_t blk_sz =  blocksize(blk);
  size_t need_sz = asize + kMetadataSize;
  size_t left_sz = blk_sz - need_sz;
  blkptr lblk = blk;
  blkptr rblk;

  if (left_sz >= MINSIZE) {         /* remainder allocable on its own */
    /* take the right block */
    rblk = block_at_offset(blk, left_sz);
    /* this also unsets PREV_INUSE for left blcok */
    set_size(rblk, need_sz);

    /* set PREV_INUSE in next block at header and footer */
    set_inuse(rblk);
    blkptr nextblk = block_at_offset(rblk, need_sz);
    set_foot(nextblk, blocksize(nextblk), blocksize_nomask(nextblk));

    /* collect the left block (a pun!) */
    set_head_size(lblk, left_sz);
    insert_into_bin(bin_index(left_sz), lblk);

    set_block_memp(rblk);
    return rblk;
  } else {                      /* remainder allocated as well */
    /* set PREV_INUSE in next block at header and footer */
    set_inuse(lblk);
    blkptr nextblk = block_at_offset(lblk, blk_sz);
    set_foot(nextblk, blocksize(nextblk), blocksize_nomask(nextblk));

    set_block_memp(lblk);

    return lblk;
  }
}

/* Allocate a free block from given non-empty bin, taking the last
   free block in the bin and returning pointer to the start of usable
   memory of the free block. */
static blkptr __alloc_from_bin(mstate m, int bidx)
{
  blkptr bin = bin_at(m, bidx);

  assert(bin != NULL);

  blkptr mblk;

  if (has_one(bin)) {           /* only one free block */
    mblk = bin;
    /* unlink */
    nil_bin_head(m, bidx);
    unset_binmap(m, bidx);
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

  set_inuse(mblk);

  /* set the next block footer too */
  blkptr nb = block_at_offset(mblk, blocksize(mblk));
  set_foot(nb, blocksize(nb), blocksize_nomask(nb));

  set_remainder(m, blocksize(mblk));

  return mblk;
}

/* `tsz` is total size for the to-be-allocated block, including both
   the usable size requested and overhead (2 boundary tags) */
static blkptr __alloc_from_top(mstate m, size_t tsz)
{
  blkptr mblk = m->top;

  size_t rsz = m->remainder - tsz;

  /* set size for alloc */
  set_size(mblk, tsz);

  /* set up new top */
  fd_top(m, mblk, tsz);
  set_head_size(m->top, rsz);

  /* set PREV_INUSE for new top at header and footer */
  set_inuse(mblk);
  set_foot(m->top, blocksize(m->top), blocksize_nomask(m->top));

  /* sub alloc size from remainder */
  dec_remainder(m, tsz);

  set_block_memp(mblk);

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
  if (size <= 0) {
    LOG("Cannot allocate size <= 0\n");
    return NULL;
  }


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

    dec_remainder(ms, blocksize(heap));

  }

  /* 4. allocate */
  return heap;

}

void my_free(void *ptr) {

  if (ptr == NULL) {
    LOG("Null pointer should not be freed.\n")
    return;
  }

  mstate m = &mmstate;
  if ((blkptr) ptr < m->lfp || (blkptr) ptr > m->hfp) {
    LOG("Given addr not in the valid range.\n")
    return;
  }

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
      else quit coalescing and insert the blcok to bin

   */

  blkptr mblk = (blkptr) ptr;
  unset_block_memp(mblk);

  unset_inuse(mblk);

  size_t blksz = blocksize(mblk);
  inc_remainder(m, blksz);

  int bidx = bin_index(blksz);

  /* insert small blcoks to small bins */
  if (in_smallbin_range(blksz)) {
    insert_into_bin(bidx, mblk);
  }
  /* TODO: Try to coalesce medium-sized or larger blocks */

  else
    insert_into_bin(bidx, mblk);

}

/* ----------------- Test main ----------------- */
int main(int argc, char* argv[])
{

  uint32 j;
  for (j = 4; j <= 20; j++) {
    int* fib = (int *) my_malloc(1UL << j);

    uint32 i;
    fib[0] = 0;
    fib[1] = 1;
    for (i = 2; i < j; i++)
      fib[i] = fib[i-1] + fib[i-2];


    my_free(fib);
  }

  /* Test medium */

  /* Test large */
  return 0;
}
