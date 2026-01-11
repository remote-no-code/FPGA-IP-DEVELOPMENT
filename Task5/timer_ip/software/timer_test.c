// timer_test.c — Heartbeat blink using Timer IP

#define TIMER_BASE   0x20001000

#define TIMER_CTRL   (*(volatile unsigned int *)(TIMER_BASE + 0x00))
#define TIMER_LOAD   (*(volatile unsigned int *)(TIMER_BASE + 0x04))
#define TIMER_VALUE  (*(volatile unsigned int *)(TIMER_BASE + 0x08))
#define TIMER_STAT   (*(volatile unsigned int *)(TIMER_BASE + 0x0C))

// For SIMULATION (fast)
#define HEARTBEAT_INTERVAL 10

//FOR HARDWARE
#define HEARTBEAT_INTERVAL 6000000   // ~0.5s @ 12 MHz


// For FPGA (uncomment when flashing)
// #define HEARTBEAT_INTERVAL 6000000   // ~0.5s @ 12 MHz

int main(void)
{
    // Load timer value
    TIMER_LOAD = HEARTBEAT_INTERVAL;

    // Enable timer: bit0=en, bit1=periodic
    TIMER_CTRL = 0x3;

    while (1) {
        // Wait for timeout
        while ((TIMER_STAT & 1) == 0)
            ;

        // Clear timeout (W1C)
        TIMER_STAT = 1;

        // Reload happens automatically in hardware (periodic mode)
    }

    return 0;
}

