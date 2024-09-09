#ifndef MYMALLOC_HEADER
#define MYMALLOC_HEADER

#include <stddef.h>
#include <stdio.h>
#include <sys/mman.h>

#ifdef ENABLE_LOG
#define LOG(...) fprintf(stderr, "[malloc] " __VA_ARGS__);
#else
#define LOG(...)
#endif

#define N_LISTS 59

#define ADD_BYTES(ptr, n) ((void *) (((char *) (ptr)) + (n)))


/** This is the Block struct, which contains all metadata needed for your
 *  explicit free list. You are allowed to modify this struct (and will need to
 *  for certain optimisations) as long as you don't move the definition from
 *  this file. **/
typedef struct Block Block;
typedef struct Block *blkptr;
typedef unsigned long uint32;
typedef unsigned long long uint64;

struct Block {

  size_t size;                  /* chunk size, including overhead */

  Block *fd;                    /* forward  */
  Block *bk;                    /* backward */
};

/* internal state */
struct malloc_state;
typedef struct malloc_state *mstate;

// Word alignment
extern const size_t kAlignment;
// Minimum allocation size (1 word)
extern const size_t kMinAllocationSize;
// Size of meta-data per Block
extern const size_t kMetadataSize;
// Maximum allocation size (128 MB)
extern const size_t kMaxAllocationSize;
// Memory size that is mmapped (64 MB)
extern const size_t kMemorySize;

void *my_malloc(size_t size);
void my_free(void *p);

/* Helper functions you are required to implement for internal testing. */

int is_free(Block *block);
size_t block_size(Block *block);

Block *get_start_block(void);
Block *get_next_block(Block *block);

Block *ptr_to_block(void *ptr);

#endif
