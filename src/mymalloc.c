#include "mymalloc.h"

// Word alignment
const size_t kAlignment = sizeof(size_t);
// Minimum allocation size (1 word)
const size_t kMinAllocationSize = kAlignment;
// Size of meta-data per Block
const size_t kMetadataSize = sizeof(Block);
// Maximum allocation size (128 MB)
const size_t kMaxAllocationSize = (128ull << 20) - kMetadataSize;
// Memory size that is mmapped (64 MB)
const size_t kMemorySize = (64ull << 20);

/* Ptr to the start of the initial heap (64MB); scoped to this file only */
static Block* INIT_HEAD = NULL;

/* Segerated lists with each list aligned to multiples of 8 bytes  */
Block* seglists[N_LISTS] = {
  NULL,                         /* [      1,       8]         */
  NULL,                         /* [      9,      16]         */
  NULL,                         /* [     17,      32]         */
  NULL,                         /* [     33,      64]         */
  NULL,                         /* [     65,     128]         */
  NULL,                         /* [    129,     256]         */
  NULL,                         /* [    257,     512]         */
  NULL,                         /* [    513,    1024] (1KB)   */
  NULL,                         /* [   1025,    2048] (2KB)   */
  NULL,                         /* [   2049,    4096] (4KB)   */
  NULL,                         /* [   4097,    8192] (8KB)   */
  NULL,                         /* [   8193,  16,384] (16KB)  */
  NULL,                         /* [ 16,385,  32,768] (32KB)  */
  NULL,                         /* [ 32,769,  65,536] (64KB)  */
  NULL,                         /* [ 65,537, 131,072] (128KB) */
  NULL,                         /* [131,073, 262,144] (256KB) */
  NULL,                         /* 256KB < any <= 64MB        */
};

/* Helper functions for block metadata */
void set_abit(Block* bptr);
void unset_abit(Block* bptr);
void set_pbit(Block* bptr);
void unset_pbit(Block* bptr);
size_t get_real_size(Block* bptr);

/* Helper functions for seglist */
size_t find_sclass_index(size_t size);
static inline size_t align(size_t size);
int need_init(void);
Block* init_heap(void);


/* core fns for allocation and deallocation */
void *my_malloc(size_t size) {
  /* FIXME: handle size > kMaxAllocationSize */
  if (size == 0)
    return NULL;

  Block* heap;
  if (need_init()) {
   heap = (Block*) init_heap();
   INIT_HEAD = heap;

   if (heap == NULL)
     return NULL;
  }

  /* 1. align size to multiples of 8 */
  size_t total_size = align(size);

  /* 2. determin size class list */
  size_t sclass_index = find_sclass_index(total_size);

  /* 3. find a best-fit free block for allocation */

  /* 4. allocate */

  return NULL;

}

void my_free(void *ptr) {
  return;
}

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
  return INIT_HEAD == NULL;
}

size_t find_sclass_index(size_t asize) /* aligned/rounded size */
{
  /* FIXME: handle the catch_all case N_LIST - 1 */
  size_t index = 0;
  size_t bound = 4;             /* 0b100 */
  while((bound <<= 1) < asize)
    index++;

  return index;
}

static inline size_t align(size_t size)
{
  const size_t mask = kAlignment - 1;
  return (size + mask) & ~mask;
}

/* Initialize the `heap', which is the memory chunk totaling 64MB to
   be used for future allocations unless it cannot satisfiy  */
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

  heap->header = kMemorySize;   /* PA: 00 */
  heap->prev = NULL;
  heap->next = NULL;

  return heap;
}

/* Test section */
int main(int argc, char* argv[])
{
  size_t need = 266;
  need = align(need);
  printf("aligned need = %lu\n", need);
  int index = find_sclass_index(need);
  printf("sclass index = %d\n", index);
  return 0;
}
