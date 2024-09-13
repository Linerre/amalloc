# Report

<!-- You should write your report in this file. Remember to check that it's
     formatted correctly in the pdf produced by the CI! -->

## Overview
The implementation described in this report aims to balance between performance and memory utilization with the following optimizations:
1. Reduced metadata size for allocated blocks.  The metadata `struct Block` takes 32 bytes in its full form but is reduced to 8 bytes for allocated blocks.

2. There are 60 circular, doubly linked lists (referred to as bins hereafter) to satisfy requests of various sizes.  Note, however, bin at index 0 does not exist and the actual index range is 1-59 (inclusive, see more info ).

3. Coalescing adjacent free blocks takes constant time using boundary tags of each free block.

Allocation and deallocation try to maintain a mutually beneficial state in the following steps:
1. When allocating, the allocator first tries to allocate from `top` because this is fastest approach, which simply moves the `top` pointer and sets up the header for the block.

2. If there is not enough space to move `top` further, the allocator then looks up a `binmap` (64-bit unsigned integer for 59 bins) for a suitable bin.  The bin contains free blocks that can match the requested size exactly (with overhead).

3. In the cases the above step also fails, the allocator start to look at non-empty bins containing larger free blocks and uses the best-fit strategy: it stops at the smallest bin that has a larger-than-request free block.

4. If all the above steps fail to find a block for allocation, it will try to request another big chunk from OS.  This, however, has not been fully implemented and tested out (see Testing section).

When deallocating, the allocator tries to respect the above priority: any to-free blocks adjacent to `top` will be merged with `top`, regardless of its left or previous block is free or not.  This means `top` is treated as a special bin not in the bins list. The next level of priority is to put small (<= 256 bytes) free blocks into the corresponding bins.  For bigger blocks, it starts trying coalescing (lowest priority).  The design principle here is to make the allocator as lazy as possible by putting off 3 relatively time-consuming tasks as much as possible: syscalls, splitting, coalescing.


## Optimisations
### Metadata Reduction
A normal `Block struct` has size of 40 bytes, this is reduced to 16 bytes for allocated blocks and two fenceposts:
![metadata-reduct](https://gitlab.cecs.anu.edu.au/u7753813/comp2310-2024-assignment-1/-/blob/main/assets/metadata-reduct.png?ref_type=heads "metadata for free and allocated blocks")

### Constant time coalescing
With footer tags in free blocks, there is no need to search any lists for consecutive free blocks.  Each time when freeing a block, the allocator will look only at the block's previous and next neighbors to see if they can be merged with the given one.  One exception is that if the next block is `top`, current free block will merge with `top` only, though this will result in two consecutive free blocks.  In any case, the coalescing takes a constant time.  This is also the approach described in the textbook and lecture.

### Multiple Free Lists
This design is directly inspired by `ptmalloc` (used in glibc).  There are 59 bins:
```
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
```

Note that since the minimal allocable chunk is of size 40 bytes (only memtadata), in fact bins[1:2] (8 bytes to 24 bytes) are all not used on 64-bit systems.  They are kept mainly for indexing convenience: given a small payload, a bin's index can be simply calculated as `payload >> 3`, for example.  The macros `bin_index` takes care of this task.

### Macros vs functions
The implementation uses a great number of function-like macros to avoid the overhead of calling short, quick procedures.  These macros are also inspired by ptmalloc's implementation.

### Requesting additional chunks from the OS
Partially done as mentioned above. A brief explanation: the global variable `heap_info` tracks how many times and bytes the allocator has asked for memory via `mmap` from OS.  Each request will be recorded into its `states` list. Thus, each block should have a tag or index that matches the heap (the big chunk) it belongs to: the reason for that index field in `Block` (metadata).

For each heap, it has its own `malloc_state` that is stored at the start of the heap.  A heap memory layout looks like below
```

 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |state|lfp| allocated chunks  |           free chunks           |tfp|
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
       ^                   ^                                     ^
       low fence          top                                   high fence
```

I have not fully tested out this feature so the relevant part is commented out at the moment.  Overall the implementation is moving towards the last few steps without significant refactoring.


## Testing
In the `mymalloc.c`, the `main` function tests merging with previous free chunk. I erased many other tests during implementation, however.

### Known bugs
`__unlink_block` at the moment fails `large`, `fragmentation`, `memset` and `bench` tests at the moment, yet I'm running out of time.  According to previous CI jobs, when I hadn't introduced `unlink`, only `memset` and `bench` failed.

Also, since the feature for multiple OS request has not been completed, this counts as a bug too.

## Benchmarking
