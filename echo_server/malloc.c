#include "malloc.h"
#include "util.h"

static unsigned char mem_block[MALLOC_MEM_SIZE] = {0};

static void *current_ptr = mem_block;

void *malloc(size_t sz)
{   
    void *ret_ptr = current_ptr;
    // update where free memory is
    unsigned char *new_head = &((unsigned char *)current_ptr)[sz];
    if (new_head > (&mem_block[MALLOC_MEM_SIZE])) {
        // went past allocated region
        return NULL;
    }

    current_ptr = new_head;

    print("malloc returned = ");
    puthex64(ret_ptr);
    print("\n");

    return ret_ptr;
}

void free(void *ptr)
{
    // leaks memory:)
    return; 
}