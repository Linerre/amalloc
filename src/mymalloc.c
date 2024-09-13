#include "mymalloc.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
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
#define MIN_BLOCK_SIZE (kMetadataSize + kAlignment)

/* The smallest aligned size that can be malloced  */
#define MINSIZE        ((MIN_BLOCK_SIZE + ALIGN_MASK) & ~ALIGN_MASK)

/* The size that free block overlaps with its malloc counterpart */
#define OVERLAP_SZ     ((size_t) 24)



/* Ceil requested size to the nearest multiples of 64MB for
   initialization.  Given the specs, this should result either 64MB or
   128MB */
#define ceilreq(req) \
(((req) + kMemorySize - (size_t) 1) / kMemorySize * kMemorySize)

/* Round up the requested size to multiple of word size */
#define req2asize(req)  (((req) + ALIGN_MASK) & ~ALIGN_MASK)

/* Like above, but use aligned size calculate the total size to allocate */
#define asz2tsize(asz) \
 ((asz) <= OVERLAP_SZ ? MINSIZE : (asz) + MINSIZE - OVERLAP_SZ)

/* Use total block size to calculate the aligned request size */
#define tsz2asize(tsz) \
 ((tsz) <= MINSIZE ? MINSIZE : (tsz) + OVERLAP_SZ - MINSIZE)

/* ------------------ Heap block operations  ------------------ */
/* Size field is OR'ed with this flag when previous adjacent block in use */
#define PREV_INUSE 0x1

/* Footer size for free blocks */
#define FTR_SZ     ((size_t) 8)

/* Header size for allocated blocks */
#define HDR_SZ     ((size_t) 8 * 2)

/* Get block size (payload + overhead), ignoring PREV_INUSE bit */
#define blocksize(b)         ((b)->size & ~ALIGN_MASK)

/* Like blocksize, but contains the PREV_INUSE bit */
#define blocksize_nomask(b)  ((b)->size)

/* Like blocksize, but excludes head and index size */
#define blocksize_nohead(b)  (blocksize(b) - HDR_SZ)

#define set_size(b, sz) ((b)->size = (sz))

/* Set size at head, without disturbing its use bit */
#define set_head_size(b, s)  ((b)->size = (((b)->size & PREV_INUSE) | (s)))

/* Set size/use field at the same time */
#define set_head(b, s)  ((b)->size = (s))

/* Set size based on `off' for footer (used only for free blocks) */
#define set_foot(b, off, sz) \
  (((blkptr) ((char *) (b) + ((off) - FTR_SZ)))->size = (sz))

/* Treat space at ptr + offset as a block */
#define block_at_offset(b, s)  ((blkptr) (((char *) (b)) + (s)))

/* Ptr to next block */
#define next_block(b)  ((blkptr) ((char *) (b) + blocksize(b)))

/* Size of the previous block (below b) */
#define prev_size(b)   (((blkptr) ((char *) (b) - FTR_SZ))->size & ~ALIGN_MASK)
#define prev_block(b)  ((blkptr) ((char *) (b) - prev_size(b)))


/* extract inuse bit of prev/next block */
#define prev_inuse(b)    ((b)->size & PREV_INUSE)
#define next_inuse(b)    (prev_inuse((block_at_offset(b, blocksize(b)))))

/* check if b is inuse via bit stored at next block */
#define inuse(b) \
  (((blkptr) ((char *) (b) + blocksize(b)))->size) & PREV_INUSE

/* set/clear block as being inuse without otherwise disturbing */
#define set_inuse(b) \
  ((blkptr) ((char *) (b) + blocksize(b)))->size |= PREV_INUSE

#define unset_inuse(b) \
  ((blkptr) ((char *) (b) + blocksize(b)))->size &= ~(PREV_INUSE)

/* Move block pointer to the start of usable memory */
#define set_memp(b) \
  ((b) = (blkptr) ((char *)b + HDR_SZ))

/* Move block pointer to the start of metadata */
#define unset_memp(b) \
  ((b) = (blkptr) ((char *)b - HDR_SZ))

/* Set block index to the curret malloc_state it belongs to */
#define set_mindex(b, i) ((b)->mindex = (i))

/* #define inc_remainder(m, sz) ((m)->remainder += (sz)) */
/* #define dec_remainder(m, sz) ((m)->remainder -= (sz)) */
/* #define set_remainder(m, sz) ((m)->remainder = (sz)) */

/* Move top forward (to higher addr) or backward (to lower addr) */
#define fd_top(m, b, sz)  ((m)->top = (blkptr)((char *)(b) + sz))
#define bk_top(m, b, sz)  ((m)->top = (blkptr)((char *)(b) - sz))

#define at_top(m, b) ((m)->top == (b))
#define at_lfp(m)    ((blkptr) ((char *) ((m)->lfp) + HDR_SZ))
#define at_hfp(m)    ((m)->hfp)

/* Check if current block is at any of the fenceposts */
#define next_to_lfp(m, b)    (b == ((m)->lfp + 1))
#define next_to_hfp(m, b)    (b == (m)->hfp)
#define next_to_top(m, b)    (next_block(b) == (m)->top)

/* Check if `top' would hit the highest boundary (hfp) */
#define checked_top(m, sz) \
  ((blkptr) ((char *)((m)->top) + (sz)) <= at_hfp(m))

/* Check next block is top */
#define next_top(m, b)  (block_at_offset(b, blocksize(b)) == (m)->top)

/* --------------------- Internal data structures --------------------- */
/*
  Segerated lists (Bins) with each list aligned to multiples of 8 bytes.

  Indexing

  List for size <= 256 bytes contain chunks of all the same size, spaced
  8 bytes apart.  Larger lists are approximately logarithmically spaced:

  -----------------------------------------------------------------
  lists in bin          spaced   Order                 Range (byte)
  -----------------------------------------------------------------
  32 bins                    8   FIFO                    1 -   256
  16 bins                   64   FIFO                  257 -  1280
   8 bins                  512   FIFO                 1281 -  5376
   2 bins                 4096   FIFO                 5377 - 13568
   1 bins          what's left   kept in size order  13568+
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
/* Handles size ranging from 32 (min) to 256 (max) */
#define MAX_SMALL_SIZE (NSMALLBINS * SMALLBIN_SPACE)

/* For index calculation, `sz' should be blocksize_nohead */
#define in_smallbin_range(sz) \
  ((sz) <= (size_t) MAX_SMALL_SIZE)

#define smallbin_index(sz) ((sz) >> 3)
#define smallbin_index_64(sz) ((((sz) - OVERLAP_SZ ) >> 3) + 1)

#define largebin_index(sz) \
  (((((sz) - MAX_SMALL_SIZE) >> 6) < 16) ? 32 + (((sz) - MAX_SMALL_SIZE) >> 6) :\
   ((((sz) - MAX_SMALL_SIZE) >> 9) < 8)  ? 48 + (((sz) - MAX_SMALL_SIZE) >> 9) :\
   ((((sz) - MAX_SMALL_SIZE) >> 12) < 2) ? 56 + (((sz) - MAX_SMALL_SIZE) >> 12) :\
   ((((sz) - MAX_SMALL_SIZE) >> 12) == 2) ? 58 : \
   N_LISTS)

#define bin_index(sz) \
  ((in_smallbin_range(sz)) ? smallbin_index(sz) : largebin_index(sz))

/* bin_at(0) does not exist */
#define bin_at(m, i)           ((m)->bins[i])
#define init_bin(m, i, b)      ((m)->bins[i] = (b))
#define set_binhead(m, i, b)   ((m)->bins[i] = (b))

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
  structure is used for bin-by-bin searching.  `binmap' is a bitvector
  recording whether bins are definitely empty so they can be skipped
  over during during traversals.  The bits cleared as soon as bins are
  empty.
 */
#define BINMAPSHIFT    6
#define BINMAPSIZE     2

/* Decide index bit  */
#define idx2bit(i)  (1ULL << (((uint64)(i) & ((1ULL << BINMAPSHIFT) - 1))))

#define set_binmap(m, i)    ((m)->binmap |= idx2bit(i))
#define unset_binmap(m, i)  ((m)->binmap &= ~(idx2bit(i)))
#define get_binmap(m, i)    ((m)->binmap & idx2bit(i))

/* ------------------- Internal state representation ------------------- */
/*
  heap_info represents the state of each mmap chunk (64MB or 128MB)
  acquired from OS.  This state is put at the start of the chunk and
  followed immediately by the leftmost fencepost.  See the memory
  layout diagram below.

  mmap chunk memory layout

  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|state| allocated chunks  |           free chunks           |tfp|
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
        ^                   ^                                 ^
        low fence          top                             high fence

  At first, all bins are empty. `top' points to the begining of first
  free area of the chunk.  Memory in this free area has not been split
  or put into any bins.  Thus, `top' represents its own special bin
  that is _not_ in the bins list.  `top' moves towards right (to
  higher addr) while allocating until it hits hfp.
 */

struct malloc_state {
  blkptr top;         /* start of free block in the entire chunk */
  blkptr lfp;         /* low fencepost, after state and before top */
  blkptr hfp;         /* high fencepost, marks end of the chunk */
  blkptr bins[NBINS]; /* see bin comments above; bin[0] does not exist */
  uint64 binmap;      /* 64-bit bitvector to track bins status */
  uint64 mindex;      /* index of current heap */
};

#define MSTATE_SZ     (sizeof(struct malloc_state))
/* Overhead for each heap = malloc_state + 2 fenceposts */
#define HEAP_OVERHEAD (MSTATE_SZ + HDR_SZ * 2)
#define MAX_STATES    100

/* The global, file-local state for all mmap-ed chunks from OS. */
static struct _heap_info {
  mstate states[MAX_STATES]; /* list of malloc_states; states[0] not exist */
  mstate current;            /* current mmap chunk */
  size_t tmsize;             /* total size of all mmaped chunks */
  uint32 tmnum;              /* total number of all mmap chunks */
} heap_info;

int malloc_initialized = -1;



/*  ---------- Heap, chunk, internal state manupilation  ---------- */

/* Request multiple of 64MB from OS */
static void* sysmmap(size_t csz)
{
  void* mem = mmap(NULL,                        /* let kernel decide */
                   csz,                         /* multiple of 64MB but <= 128 */
                   PROT_READ | PROT_WRITE,      /* prot              */
                   MAP_PRIVATE | MAP_ANONYMOUS, /* flags             */
                   0,                           /* fd                */
                   0                            /* offset            */
                   );

  if (mem == MAP_FAILED) {
    LOG("mmap failed: %s\n", strerror(errno));
    return NULL;
  }

  return mem;
}

/* Set up malloc_state for a given mmap chunk */
static void __init_malloc_state(mstate m, size_t csz)
{
  hinfo h = &heap_info;

  /* register this mstate in global heap info */
  h->tmsize += csz;
  h->tmnum += 1;
  h->states[h->tmnum] = m;
  h->current = m;

  /* init mindex and binmap */
  m->mindex = h->tmnum;
  m->binmap = 0;

  /* init bins */
  size_t i;
  size_t t = N_LISTS + 1;
  for (i = 0; i < t; i++)
    m->bins[i] = NULL;


  /* init lfp, hfp.  Note: fenceposts are shrinked to have
     `size' and `mindex' fields only to save space  */
  m->lfp = (blkptr) ((char *)m + MSTATE_SZ);
  m->lfp->mindex = m->mindex;
  m->hfp = (blkptr) ((char *)m + csz - HDR_SZ);
  m->hfp->mindex = m->mindex;

  /* init top */
  m->top = (blkptr) ((char *)m->lfp + HDR_SZ) ;
  m->top->mindex = m->mindex;
  m->top->size = csz - HEAP_OVERHEAD;
}

/*
  Initialize the `heap', which is the memory chunk totaling ceiled
  size, to be used for future allocations unless it can no longer
  satisfiy
 */
static blkptr init_malloc(size_t csz)
{
  mstate m = sysmmap(csz);

  if (m == NULL)
    return NULL;

  __init_malloc_state(m, csz);

  /* Mark initialization */
  malloc_initialized = 0;

  return m->top;
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
static void __insert_normal_bins(mstate m, blkptr mblk, int bidx)
{
  blkptr bin = bin_at(m, bidx);

  if (is_empty(bin)){
    init_bin(m, bidx, mblk);
    bin = mblk;
    bin->fd = bin->bk = mblk;
    set_binmap(m, bidx);
  } else if (has_one(bin)) {    /* 1 free blocks only */
    mblk->fd = bin;
    mblk->bk = bin;
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
static void __insert_last_bin(mstate m, blkptr mblk)
{
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
static void insert_into_bin(mstate m, blkptr mblk, int bidx) {

  if (bidx == N_LISTS)
    __insert_last_bin(m, mblk);
  else
    __insert_normal_bins(m, mblk, bidx);
}

/* Split the larger `blk' at higher addr end and return the rightmost
   part for malloc, putting remaining left (low-addr) part back to a
   suitable bin. */
static blkptr split_block(mstate m, blkptr blk, size_t asize)
{
  size_t blk_sz =  blocksize(blk);
  size_t need_sz = asz2tsize(asize);
  size_t left_sz = blk_sz - need_sz;
  blkptr lblk = blk;
  blkptr rblk;

  if (left_sz >= MINSIZE) {         /* remainder allocable on its own */
    /* take the right block */
    rblk = block_at_offset(blk, left_sz);
    /* this also unsets PREV_INUSE for left blcok */
    set_size(rblk, need_sz);
    set_mindex(rblk, m->mindex);


    /* set PREV_INUSE in next block at header and footer */
    set_inuse(rblk);
    blkptr nextblk = next_block(rblk);
    set_foot(nextblk, blocksize(nextblk), blocksize_nomask(nextblk));

    /* collect the left block (a pun!) */
    set_head_size(lblk, left_sz);
    insert_into_bin(m, lblk, bin_index(left_sz));

    set_memp(rblk);
    return rblk;
  } else {                      /* remainder allocated as well */
    /* set PREV_INUSE in next block at header and footer */
    set_inuse(blk);
    blkptr nextblk = next_block(blk);
    set_foot(nextblk, blocksize(nextblk), blocksize_nomask(nextblk));

    set_memp(blk);

    return blk;
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

  set_mindex(mblk, m->mindex);
  set_inuse(mblk);

  /* set the next block footer too */
  blkptr nb = next_block(mblk);
  set_foot(nb, blocksize(nb), blocksize_nomask(nb));


  return mblk;
}

/* `tsz` is total size for the to-be-allocated block, including both
   the usable size requested and overhead (size + mindex) */
static blkptr __alloc_from_top(mstate m, size_t tsz)
{
  blkptr mblk = m->top;
  size_t rsz = blocksize(m->top) - tsz;

  /* set head size for alloc but inherit top's PREV_INUSE bit */
  set_head_size(mblk, tsz);
  set_mindex(mblk, m->mindex);

  /* set up new top */
  fd_top(m, mblk, tsz);
  set_size(m->top, rsz);
  set_mindex(m->top, m->mindex);

  /* set PREV_INUSE for new top at header and footer */
  set_inuse(mblk);
  set_foot(m->top, blocksize(m->top), blocksize_nomask(m->top));
  assert(m->top->size % 2 != 0);

  set_memp(mblk);

  return mblk;
}

/* Try to find a larger free block from the bins after the current
   `idx' for malloc.  The found block will be split first (see
   `split_block').  Return the malloc block on success and NULL on
   failure. */
static blkptr __try_alloc_from_nextbin(mstate m, int bidx, size_t basize)
{
  int nbidx = find_next_nonempty_bin(m, bidx);

  if (nbidx > 0) {
    blkptr mblk = bin_at(m, nbidx);
    return split_block(m, mblk, basize);
  }

  return NULL;
}

static void __unlink_block(mstate m, blkptr blk)
{
  size_t nxsz = tsz2asize(blocksize(blk));
  int nbidx = bin_index(nxsz);

  blkptr bin = bin_at(m, nbidx);

  assert(bin != NULL);

  blkptr fd = blk->fd;
  blkptr bk = blk->bk;

  if (has_one(bin)) {           /* 1 free block and empty after */
    assert(bin == blk);         /* blk should be bin head */
    bin->fd = bin->bk = NULL;
    nil_bin_head(m, nbidx);
    unset_binmap(m, nbidx);
  } else {                      /* 2 or more free blocks */
    fd->bk = bk;
    bk->fd = fd;
    if (blk == bin) {           /* blk was head and fd is new head */
      init_bin(m, nbidx, fd);
    }
  }
}

/* Merge given block with top without any bin insertions */
static void __merge_with_top(mstate m, blkptr mblk)
{
  /* new head should maintain the PREV_INUSE bit */
  size_t new_tsz = blocksize_nomask(mblk) + blocksize(m->top);

  /* retain the PREV_INUSE bit */
  set_head_size(mblk, new_tsz);
  set_foot(mblk, new_tsz, new_tsz);
  set_mindex(mblk, m->mindex);

  /* top should _always_ next to hfp so no need to unset inuse */

  /* update top */
  m->top = mblk;
}

/* Like above, but the next is not top */
static void __merge_with_next(mstate m, blkptr mblk, blkptr nextblk)
{
  /* new head should maintain the PREV_INUSE bit */
  size_t new_tsz = blocksize_nomask(mblk) + blocksize(nextblk);
  size_t new_asz = blocksize_nohead(mblk) + blocksize(nextblk);

  set_head_size(mblk, new_tsz);
  set_foot(mblk, new_tsz, new_tsz);
  set_mindex(mblk, m->mindex);

  /* update new next block prev_inuse bit in header and leave
     its footer untouched as it is inuse thus without footer */
  unset_inuse(mblk);

  /* unlink next now */
  __unlink_block(m, nextblk);

  /* insert coalesced new block into suitable bin */
  int new_bidx = bin_index(new_asz);
  insert_into_bin(m, mblk, new_bidx);
}

static void __merge_with_prev(mstate m, blkptr mblk, blkptr prevblk)
{

  /* unlink prev first otherwise wrong size will result in wrong bidx */
  __unlink_block(m, prevblk);

  size_t new_tsz = blocksize(prevblk) + blocksize(mblk);
  size_t new_asz = blocksize_nohead(prevblk) + blocksize(mblk);

  /* retian PREV_INUSE bit */
  set_head_size(prevblk, new_tsz);
  set_foot(prevblk, new_tsz, new_tsz);
  set_mindex(prevblk, m->mindex);

  unset_inuse(prevblk);

  int new_bidx = bin_index(new_asz);
  insert_into_bin(m, prevblk, new_bidx);
}

static void
__merge_with_both(mstate m, blkptr mblk, blkptr prevblk, blkptr nextblk)
{
  /* unlink both prev and next */
  __unlink_block(m, prevblk);
  __unlink_block(m, nextblk);

  size_t new_tsz = blocksize(prevblk) + blocksize(mblk) + blocksize(nextblk);
  size_t new_asz = blocksize_nohead(prevblk) + blocksize(mblk) + blocksize(nextblk);

  /* retian PREV_INUSE bit */
  set_head_size(prevblk, new_tsz);
  set_foot(prevblk, new_tsz, new_tsz);
  set_mindex(prevblk, m->mindex);

  unset_inuse(prevblk);

  int new_bidx = bin_index(new_asz);
  insert_into_bin(m, mblk, new_bidx);

}

static void coalesce(mstate m, blkptr mblk, int bidx)
{
  blkptr nextblk = next_block(mblk);
  blkptr prevblk = prev_block(mblk);

  if (prev_inuse(mblk) && next_inuse(nextblk)) {       /* case 1 */
    insert_into_bin(m, mblk, bidx);
  }

  else if (prev_inuse(mblk) && !next_inuse(nextblk)) { /* case 2 */
    /* if this is last free block right before the rightmost
       fencepost, insert it to a bin as it cannot coalesce into end;
       otherwise merge with the next (right) block
     */
    if (next_to_hfp(m, nextblk))
      insert_into_bin(m, mblk, bidx);

    else
      __merge_with_next(m, mblk, nextblk);
  }

  else if (!prev_inuse(mblk) && next_inuse(nextblk)) { /* case 3 */
    /* if this is the very 1st free block immediately after leftmost
       fencepost, insert it to a bin as it cannot coalesce into end;
       otherwise merge with the next (right) block
     */
    /* blkptr prevblk = prev_block(mblk); */
    if (next_to_lfp(m, prevblk))
      insert_into_bin(m, mblk, bidx);

    else
      __merge_with_prev(m, mblk, prevblk);
  }

  else  {                                              /* case 4 */
    __merge_with_both(m, mblk, prevblk, nextblk);
  }
}



/* ----------------- Required helper functions ----------------- */
/** These are helper functions you are required to implement for
 *  internal testing purposes. Depending on the optimisations you
 *  implement, you will need to update these functions yourself.
 **/

/* Returns 1 if the given block is free, 0 if not. */
int is_free(blkptr block) {
  return !(inuse(block));
}

/* Returns the size of the given block */
size_t block_size(blkptr block) {
  return blocksize(block);
}

/* Returns the first block in memory (excluding fenceposts)
   Note: this implementation can _only_ return the first block
   of current heap if there are multiple ones.
 */
blkptr get_start_block(void) {

  hinfo h = &heap_info;
  if (h == NULL) {
    LOG("Failed to get first block: unkonwn heap info\n");
    return NULL;
  }

  if (h->current == NULL) {
    LOG("Failed to get first block: no current malloc state\n");
    return NULL;
  }

  mstate m = h->current;

  if (m->lfp == NULL) {
    LOG("Failed to get first block: current malloc has no left fencepost\n");
    return NULL;
  }

  return block_at_offset(m->lfp, HDR_SZ);
}

/* Returns the next block in memory.  Note: this implementation can
   _only_ return the next block of current heap if there are multiple
   ones.  Fenceposts are excluded (not treated as normal blocks) */
blkptr get_next_block(blkptr block) {
  hinfo h = &heap_info;
  if (h == NULL) {
    LOG("Failed to get next block: unkonwn heap info\n");
    return NULL;
  }

  if (h->current == NULL) {
    LOG("Failed to get next block: no current malloc state\n");
    return NULL;
  }

  mstate m = h->current;

  if (m->hfp == NULL) {
    LOG("Failed to get next block: current malloc has no right fencepost\n");
    return NULL;
  }

  if (next_to_hfp(m, block))
    return NULL;
  else
    return next_block(block);
}

/* Given a ptr assumed to be returned from a previous call to `my_malloc`,
   return a pointer to the start of the metadata block. */
blkptr ptr_to_block(void *ptr) {
  return (blkptr) ((char *)ptr - HDR_SZ);
}

/* ----------------- Core malloc/free functions ----------------- */
/*
  `size' is user requested when calling this allocator. For a single
  call of this function each time, 1B <= size <= 128MB;
 */
void* my_malloc(size_t size)
{
  if (size == 0 || size > kMaxAllocationSize) {
    LOG("Cannot allocate size <= 0 or size > 128MB at a time\n");
    return NULL;
  }

  hinfo h = &heap_info;         /* global heap info */
  mstate m;                     /* current malloc state */
  blkptr top;                   /* current malloc block top */
  blkptr mblk;                  /* block for allocation */

  if (malloc_initialized < 0) {
    top = init_malloc(ceilreq(size + HEAP_OVERHEAD));

    if (top == NULL)
      return NULL;
  }

  if (h->current == NULL) {
    LOG("Invalid malloc state\n");
    return NULL;
  }

  m = h->current;
  /* redaundant but harmless */
  top = m->top;

  /* 1. align requested size to multiple of 8 and check for minsize */
  size_t blk_asz = req2asize(size);
  size_t total_asz = asz2tsize(blk_asz);

  /* 2. determin size class list */
  int bidx = bin_index(blk_asz);

  if (checked_top(m, total_asz)) {
    return __alloc_from_top(m, total_asz);
  }
  else if (get_binmap(m, bidx)) {
    return __alloc_from_bin(m, bidx);
  }
  else {
    /* try to alloc from the next larger non-empty bin */
    mblk =  __try_alloc_from_nextbin(m, bidx, blk_asz);

    /* TODO: perhaps scanning the entire heap and collect any free
       blocks would be the last hope before new mmap.  This is garbage
       collection feature yet to be implemented */
    if (mblk == NULL) {
      /* init a new malloc heap and switch to it */
      /* blkptr new_top = init_malloc(ceilreq(size)); */
      /* if (new_top == NULL) */
      /*   return NULL; */
      /*  */
      /* /\* try to malloc using this new heap *\/ */
      /* return my_malloc(size); */
    }
    /* else */

  }

  /* 4. allocate */
  return mblk;

}

void my_free(void *ptr) {

  if (ptr == NULL) {
    LOG("Free failed: cannot free null pointer.\n")
    return;
  }

  hinfo h = &heap_info;         /* global heap info */
  if (h->current == NULL || h->tmsize == 0 || h->tmnum == 0) {
    LOG("No current heap and address not recognized \n");
    LOG("Falling back to syscall free ...\n");

    errno = 0;
    /* try to free using `free' */
    free(ptr);
    if (errno < 0)
      LOG("Free faild:  syscall free failed at given addr\n");

    return;

  }


  mstate m;                     /* malloc state for this free */
  blkptr mblk = (blkptr) ptr;   /* cast to block pointer */

  unset_memp(mblk);             /* restore malloc block head position */
  uint64 midx = mblk->mindex;   /* block's memory scope */

  if (h->states[midx] == NULL) {
    LOG("Free failed: invalid block index: %llu.\n", midx);
    return;
  }

  /* Check out the correct malloc_state */
  m = h->states[midx];

  /* In case user modifies the block index by mistake or intentionally */
  if (mblk < at_lfp(m) || mblk > at_hfp(m)) {
    LOG("Free failed: block address not in valid range.\n");
    return;
  }

  /* Check double free error */
  if (!(inuse(mblk))) {
    LOG("Double free error\n");
    return;
  }

  /* retore footer for the given block */
  set_foot(mblk, blocksize(mblk), blocksize_nomask(mblk));
  /* update header and footer for the next block */
  unset_inuse(mblk);
  blkptr nextblk = next_block(mblk);
  set_foot(nextblk, blocksize(nextblk), blocksize(nextblk));

  size_t blktsz = blocksize(mblk);
  size_t blkasz = blktsz == MINSIZE ? MINSIZE : blocksize_nohead(mblk);

  int bidx = bin_index(blkasz);

  /* align with alloc priority but will result in two consecutive free
     blocks.  In such cases, we think the two blocks belong to two
     different bins */
  if (next_to_top(m, mblk))
    __merge_with_top(m, mblk);


  else if (in_smallbin_range(blkasz))
    insert_into_bin(m, mblk, bidx);


  else
    coalesce(m, mblk, bidx);
}

/* ----------------- Test main ----------------- */
int main(int argc, char* argv[])
{

  void *fib = my_malloc(100);   /* round to 104 */
  assert(fib != NULL);

  /* Test small */
  void *fib1 =  my_malloc(123);
  assert(fib1 != NULL);
  memset(fib1, 0, 123);

  /* Test medium */
  int* fib2 = (int *) my_malloc(4011);
  assert(fib2 != NULL);

  /* Test large */
  int* fib3 = (int *) my_malloc(1UL << 20);
  assert(fib3 != NULL);

  my_free(fib1);                /* expect to be put in a bin */
  my_free(fib2);                /* expect to merge with fib1 */

  return 0;
}
