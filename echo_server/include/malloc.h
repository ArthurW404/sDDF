#pragma once

#include <stddef.h>
#include <stdint.h>

#define MALLOC_MEM_SIZE 0x800000 // 16MB ish

void *malloc(size_t sz);
void free(void *ptr);