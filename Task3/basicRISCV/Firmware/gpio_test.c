#include "io.h"
#include "print.h"
#include <stdarg.h>

#define GPIO_BASE 0x20000000
#define GPIO_DATA (*(volatile unsigned int *)(GPIO_BASE + 0x00))
#define GPIO_DIR  (*(volatile unsigned int *)(GPIO_BASE + 0x04))
#define GPIO_READ (*(volatile unsigned int *)(GPIO_BASE + 0x08))

int main() {
    print_string("GPIO Task-3 Test\n");

    GPIO_DIR  = 0x0000000F;   // GPIO[3:0] output
    GPIO_DATA = 0x00000005;   // 0101

    unsigned int r = GPIO_READ;

    print_string("GPIO_READ = ");
    print_hex(r);
    print_string("\n");

    while (1);
}
