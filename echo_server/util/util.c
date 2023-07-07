#include "malloc.h"
#include "util.h"
#include "printf.h"


// needed for printf
void _putchar(char character) {
    putC(character);
}

void
dump_packet(void *packet_ptr, int len)
{
    uint8_t *packet = (uint8_t *) packet_ptr;
    for (unsigned i = 0; i < len; i++) {
        sel4cp_dbg_putc(hexchar((packet[i] >> 4) & 0xf));
        sel4cp_dbg_putc(hexchar(packet[i] & 0xf));
        if (i < len - 1) {
            sel4cp_dbg_putc(' ');
        }
    } 
}

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
    print("malloc size = ");
    puthex64(sz);
    print("\n");
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


void udelay(uint32_t us)
{
    volatile int i;
    for (; us > 0; us--) {
        for (i = 0; i < 100; i++) {
        }
    }
}

unsigned long simple_strtoul(const char *cp, char **endp,
                             unsigned int base)
{
    unsigned long result = 0;
    unsigned long value;

    if (*cp == '0') {
        cp++;
        if ((*cp == 'x') && isxdigit(cp[1])) {
            base = 16;
            cp++;
        }

        if (!base) {
            base = 8;
        }
    }

    if (!base) {
        base = 10;
    }

    while (isxdigit(*cp) && (value = isdigit(*cp) ? *cp - '0' : (islower(*cp)
                                                                 ? toupper(*cp) : *cp) - 'A' + 10) < base) {
        result = result * base + value;
        cp++;
    }

    if (endp) {
        *endp = (char *)cp;
    }

    return result;
}
