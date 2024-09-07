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

/* TODO:
   1. Free chunks are stored in circular doubly-linked lists
   2.
 */

/* ------------------ Global, file-local constants ------------------ */
/* The aligment mask */
static const size_t ALIGN_MASK = kAlignment - 1;
/* The smallest possible chunk */
static const size_t MIN_CHUNK_SIZE = kMetadataSize + kMinAllocationSize + kAlignment;
/* The smallest size allocable is an aligned minial chunk */
static const size_t MINSIZE = (MIN_CHUNK_SIZE + ALIGN_MASK) & ~ALIGN_MASK;

/* Small bin upper bound (256B) */
static const size_t SBINBOUND = (1UL << 8);
/* Medium bin upper bound (1KB) */
static const size_t MBINBOUND = (1UL << 10);
/* Large bin upper bound (4KB)  */
static const size_t LBINBOUND = (1UL << 10);

static Block *HEAP_TOP = NULL;

/* --------------------- Internal data structures --------------------- */
/*
  Segerated lists (Bins) with each list aligned to multiples of 8 bytes

  Indexing

  List for size < 256 bytes contain chunks of all the same size, spaced
  8 bytes apart.  Larger lists are approximately logarithmically spaced:

  32 bins of size            8
  16 bins of size           64
   4 bins of size          512
   2 bins of size         4096
   1 bins of size  what's left
 */
Block* seglists[N_LISTS];

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

  /* Top end of the allocated 64MB from one mmp call */
  Block *top;

  /* the remaining, allocable space */
  size_t remainder;
} mstate;

/* --------------------- Helper functions --------------------- */
/* TODO: convert these simple functions to funciton-like macros */
/** Helper functions for block metadata **/
void set_abit(Block* bptr)
{
  bptr->header |= 1;
}

void unset_abit(Block* bptr)
{
  bptr->header &= ~1;
}

void set_pbit(Block* bptr)
{
  bptr->header |= (1 << 1);
}

void unset_pbit(Block* bptr)
{
  bptr->header &= ~(1 << 1);
}

size_t get_real_size(Block* bptr)
{
  return bptr->header & ~2;
}

/* Helper functions for seglists */
int need_init(void)
{
  return HEAP_TOP == NULL;
}

size_t find_sclass(size_t asize) /* aligned size */
{
  /* Best fit for request larger than 256KB */
  if (asize >= ((size_t)1<<8))
    return 16;

  size_t class = 0;
  size_t bound = 4;             /* 0b100 */
  while((bound <<= 1) < asize)
    class++;

  return class;
}

static inline size_t align(size_t size)
{
  const size_t mask = kAlignment - 1;
  return (size + mask) & ~mask;
}

/*
  Initialize the `heap', which is the memory chunk totaling 64MB
  to be used for future allocations unless it can no longer satisfiy
 */
Block* init_heap(void)
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

  struct malloc_state *mst = &mstate;

  /* Set fenceposts at both start and end of the heap */
  Block *dm_head = heap;
  Block *dm_tail = (Block*) ((char*)heap + (kMemorySize - kMetadataSize));
  dm_head->header = 0;
  dm_head->prev = NULL;
  dm_head->next = heap + 1;
  dm_tail->header = 0;
  dm_tail->next = NULL;
  dm_tail->prev = dm_tail - 1;

  /* TODO: store top at dummy head or the first allocable chunk? */
  mst->top = heap;

  /* Record remaining space */
  mst->remainder = kMemorySize - (2 * kMetadataSize);

  /* TODO: how do i know where to allocate and split for the first time? */
  return heap;
}

/* ----------------- Required helper functions ----------------- */
/** These are helper functions you are required to implement for
 *  internal testing purposes. Depending on the optimisations you
 *  implement, you will need to update these functions yourself.
 **/

/* Returns 1 if the given block is free, 0 if not. */
int is_free(Block *block) {
  return block->header & 1;
}

/* Returns the size of the given block */
size_t block_size(Block *block) {
  return get_real_size(block);
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
  return ADD_BYTES(ptr, -((ssize_t) kMetadataSize));
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
  if (need_init()) {
   heap = (Block*) init_heap();

   if (heap == NULL)
     return NULL;
  }

  /* 1. align size(=requested + header) to multiples of 8 */
  size_t total_size = align(size + kMetadataSize);

  /* 2. determin size class list */
  size_t sclass_index = find_sclass(total_size);

  /* 3. find a best-fit free block for allocation */
  Block* cls_head = seglists[sclass_index];
  if (cls_head == NULL) {       /* first use of this size class */
    /* init dummy fenceposts */

  } else {

  }


  /* 4. allocate */

  return NULL;

}

void my_free(void *ptr) {
  return;
}

/* ----------------- Test main ----------------- */
int main(int argc, char* argv[])
{

  Block* bptr = init_heap();


  size_t need = 64ull<<20;
  need = align(need);
  printf("aligned need = %lu\n", need);
  int index = find_sclass(need);
  printf("sclass index = %d\n", index);

  return 0;
}
