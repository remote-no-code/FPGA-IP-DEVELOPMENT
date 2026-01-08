#include "io.h"
#include "print.h"

#define GPIO_BASE 0x20000000
#define GPIO_DATA (*(volatile unsigned int *)(GPIO_BASE + 0x00))
#define GPIO_DIR  (*(volatile unsigned int *)(GPIO_BASE + 0x04))
#define GPIO_READ (*(volatile unsigned int *)(GPIO_BASE + 0x08))

int main() {
    // Configure GPIO[7:0] as output
    GPIO_DIR = 0x000000FF;

    // Write test pattern
    GPIO_DATA = 0x000000A5;

    // Read back
    unsigned int r = GPIO_READ;

    // UART output
    print_hex(r);        // prints 000000A5
    print_string("\n");

    while (1);           // stay here
}
