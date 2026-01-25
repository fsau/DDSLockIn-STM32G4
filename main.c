#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <stdint.h>
#include "blink.h"

extern uint32_t _ccm_text_loadaddr;
extern uint32_t _ccm_text_start;
extern uint32_t _ccm_text_end;

static void ccm_init(void)
{
    uint32_t *src = &_ccm_text_loadaddr;
    uint32_t *dst = &_ccm_text_start;

    while (dst < &_ccm_text_end)
        *dst++ = *src++;
}

int main(void)
{
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_170MHZ]);

    /* Enable GPIOC clock */
    rcc_periph_clock_enable(RCC_GPIOC);

    /* PC13 as push-pull output */
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT,
                    GPIO_PUPD_NONE, GPIO6);

    /* Optional: slower edge, less EMI */
    gpio_set_output_options(GPIOC,
                            GPIO_OTYPE_PP,
                            GPIO_OSPEED_2MHZ,
                            GPIO6);
    while(1){
        blink(1600000);
    }

    return 0;
}

void nmi_handler(void)
{    
    while(1){
        blink(1600000);
    }
}
