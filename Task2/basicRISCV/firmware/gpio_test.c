#define GPIO_ADDR 0x00400020

extern void print_hex(unsigned int);
extern void print_string(const char *);

int main() {
    volatile unsigned int *gpio =
        (volatile unsigned int *) GPIO_ADDR;

    *gpio = 0xA5;               // WRITE
    unsigned int val = *gpio;   // READ

    print_string("GPIO readback = ");
    print_hex(val);
    print_string("\n");

    while (1);
}
