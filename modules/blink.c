#include <libopencm3/stm32/gpio.h>
#include <stdint.h>

void blink(int k)
{
    gpio_set(GPIOC, GPIO6);
    for (int i = 0; i < k; i++)
    {
        __asm__("nop");
    }
    gpio_clear(GPIOC, GPIO6);
    for (int i = 0; i < k; i++)
    {
        __asm__("nop");
    }
}
