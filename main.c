#include <stdio.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/crs.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/g4/syscfg.h>

usbd_device *usbd_dev;

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
} };

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
} };

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	 }
};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors)
} };

static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
} };

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
} };

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Black Sphere Technologies",
	"CDC-ACM Demo",
	"DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		return USBD_REQ_HANDLED;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding)) {
			return USBD_REQ_NOTSUPP;
		}

		return USBD_REQ_HANDLED;
	}
	return USBD_REQ_NOTSUPP;
}

#define disable_irq() __asm__("cpsid i")
#define enable_irq()  __asm__("cpsie i")
#define set_msp(addr) __asm__("msr msp, %0" : : "r" (addr))

/* Reset STM32G4 clocks to default (HSI16, PLL off) */
void reset_clocks_to_default(void)
{
    /* 1. Switch SYSCLK to HSI16 */
    RCC_CR |= RCC_CR_HSION;                  // enable HSI16
    while (!(RCC_CR & RCC_CR_HSIRDY));      // wait until ready

    RCC_CFGR &= ~RCC_CFGR_SW_MASK;          // select HSI16 as SYSCLK
    while ((RCC_CFGR & RCC_CFGR_SWS_MASK) != 0x00); // check SYSCLK switch

    /* 2. Turn off PLL, HSE, HSI48 */
    RCC_CR &= ~RCC_CR_PLLON;
    RCC_CR &= ~RCC_CR_HSEON;
    RCC_CRRCR &= ~RCC_CRRCR_HSI48ON;

    /* 3. Reset prescalers */
    RCC_CFGR &= ~RCC_CFGR_HPRE_MASK;        // HCLK = SYSCLK
    RCC_CFGR &= ~RCC_CFGR_PPRE1_MASK;       // PCLK1 = HCLK
    RCC_CFGR &= ~RCC_CFGR_PPRE2_MASK;       // PCLK2 = HCLK

    /* 4. Disable USB & CRS clocks */
    RCC_AHB2ENR &= ~RCC_APB1ENR1_USBEN;
    RCC_APB1ENR1 &= ~RCC_APB1ENR1_CRSEN;

    /* 5. Optional: reset RCC interrupt registers */
    RCC_CIER = 0x00000000;

    /* 6. Reset GPIOs if needed */
    // Example: set USB pins to input to avoid glitches
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO11 | GPIO12);
}

// static void jump_to_bootloader(void)
// {
//     volatile uint32_t *sysmem_vec = (uint32_t *)0x1FF00000UL;

//     /* Deinit USB completely */
//     nvic_disable_irq(NVIC_USB_HP_IRQ);
//     nvic_disable_irq(NVIC_USB_LP_IRQ);
//     nvic_disable_irq(NVIC_USB_WAKEUP_IRQ);

//     RCC_APB1RSTR1 |= RCC_APB1RSTR1_USBRST;
//     __asm__("nop");
//     RCC_APB1RSTR1 &= ~RCC_APB1RSTR1_USBRST;

//     gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO11 | GPIO12);
//     rcc_periph_clock_disable(RCC_CRS);
// 	rcc_periph_clock_disable(RCC_GPIOA);
//     rcc_periph_clock_disable(RCC_USB);

//     reset_clocks_to_default();
//     /* Disable all interrupts and jump */
//     disable_irq();
//     set_msp(sysmem_vec[0]);

//     for(volatile int i=0;i<50000;i++);

//     void (*bootloader_entry)(void) = (void (*)(void))sysmem_vec[1];
//     bootloader_entry();

//     while (1); // fallback, should never reach here
// }

static volatile uint32_t ms_ticks = 0;

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    if (len) {
        if (buf[0] == 'P' || buf[0] == 'p') {

            // uint32_t pllcfgr = RCC_PLLCFGR;

            // // PLLSRC bits [1:0] para STM32G4 (ver datasheet):
            // uint32_t pllsrc_bits = (pllcfgr >> 0) & 0x3; // ou RCC_PLLCFGR_PLLSRC_SHIFT se definido

            // const char *pll_src_str;
            // if (pllsrc_bits == 0)       // HSI
            //     pll_src_str = "HSI";
            // else if (pllsrc_bits == 1)  // HSE
            //     pll_src_str = "HSE";
            // else
            //     pll_src_str = "Unknown";

            // /* PLLM: 1..16 */
            // uint32_t pllm = ((pllcfgr >> RCC_PLLCFGR_PLLM_SHIFT) & RCC_PLLCFGR_PLLM_MASK) + 1;

            // /* PLLN: 8..127 */
            // uint32_t plln = (pllcfgr >> RCC_PLLCFGR_PLLN_SHIFT) & RCC_PLLCFGR_PLLN_MASK;

            // /* PLLP: especial, bit17 indica 17 ou 7 */
            // uint32_t pllp = (pllcfgr & RCC_PLLCFGR_PLLP) ? 17 : 7;

            // /* PLLQ: 2,4,6,8 */
            // uint32_t pllq_index = (pllcfgr >> RCC_PLLCFGR_PLLQ_SHIFT) & RCC_PLLCFGR_PLLQ_MASK;
            // uint32_t pllq = 2 + pllq_index * 2;

            // /* PLLR: 2,4,6,8 */
            // uint32_t pllr_index = (pllcfgr >> RCC_PLLCFGR_PLLR_SHIFT) & RCC_PLLCFGR_PLLR_MASK;
            // uint32_t pllr = 2 + pllr_index * 2;

            // char msg[160];
            // int msg_len = snprintf(msg, sizeof(msg),
            //     "PLL Source: %s\r\nPLLM=%lu, PLLN=%lu, PLLP=%lu, PLLQ=%lu, PLLR=%lu\r\n",
            //     pll_src_str, pllm, plln, pllp, pllq, pllr
            // );

            char msg[160];
            int msg_len = snprintf(msg, sizeof(msg),
                "t = %lu\r\n",ms_ticks);
            usbd_ep_write_packet(usbd_dev, 0x82, msg, msg_len);
        } else if (buf[0] == 'r' || buf[0] == 'R') {
            char msg[] = "Rebooting into bootloader...\r\n";
            usbd_ep_write_packet(usbd_dev, 0x82, msg, sizeof(msg)-1);
            // jump_to_bootloader();
        } else {
            // normal echo + increment
            for (int i = 0; i < len; i++)
                buf[i]++;

            while (usbd_ep_write_packet(usbd_dev, 0x82, buf, len) == 0);
        }
    }
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
			cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}

// SysTick interrupt handler
void sys_tick_handler(void)
{
    ms_ticks++;
}

// Initialize SysTick for millisecond ticks
void systick_setup(uint32_t sysclk_hz)
{
    // SysTick = SYSCLK / 1000 -> 1ms tick
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(sysclk_hz / 1000 - 1);
    systick_clear();
    systick_counter_enable();
    systick_interrupt_enable();
}

// Delay in milliseconds
void delay_ms(uint32_t ms)
{
    uint32_t start = ms_ticks;
    while ((ms_ticks - start) < ms)
        __asm__("nop");
}

int main(void)
{
	// usbd_device *usbd_dev;
    struct rcc_clock_scale pllconfig = rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_170MHZ];
	rcc_clock_setup_pll(&pllconfig);

    // Enable HSI48
    RCC_CRRCR |= RCC_CRRCR_HSI48ON;
    // Wait until ready
    while (!(RCC_CRRCR & RCC_CRRCR_HSI48RDY));   // enable HSI48
    rcc_set_clock48_source(RCC_CCIPR_CLK48SEL_HSI48);

	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_CRS);
    crs_autotrim_usb_enable();

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);  // USB_FS

    // /* Reset USB */
    // RCC_APB1RSTR1 |= RCC_APB1RSTR1_USBRST;
    // __asm__("nop"); // short delay
    // RCC_APB1RSTR1 &= ~RCC_APB1RSTR1_USBRST;

    usbd_dev = usbd_init(&st_usbfs_v2_usb_driver,
                        &dev,
                        &config,
                        usb_strings, 3,
                        usbd_control_buffer,
                        sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);


    systick_setup(rcc_ahb_frequency);
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

	while (1) {
		usbd_poll(usbd_dev);

    // ms_ticks++;
        // gpio_toggle(GPIOC, GPIO6);
        // delay_ms(1000);
	}
}

// #include <libopencm3/stm32/rcc.h>
// #include <libopencm3/stm32/gpio.h>
// #include <stdint.h>
// #include "blink.h"

// extern uint32_t _ccm_text_loadaddr;
// extern uint32_t _ccm_text_start;
// extern uint32_t _ccm_text_end;

// static void ccm_init(void)
// {
//     uint32_t *src = &_ccm_text_loadaddr;
//     uint32_t *dst = &_ccm_text_start;

//     while (dst < &_ccm_text_end)
//         *dst++ = *src++;
// }

// int main(void)
// {
//     rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_170MHZ]);
//     ccm_init();

//     /* Enable GPIOC clock */
//     rcc_periph_clock_enable(RCC_GPIOC);

//     /* PC13 as push-pull output */
//     gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT,
//                     GPIO_PUPD_NONE, GPIO6);

//     /* Optional: slower edge, less EMI */
//     gpio_set_output_options(GPIOC,
//                             GPIO_OTYPE_PP,
//                             GPIO_OSPEED_2MHZ,
//                             GPIO6);
//     while(1){
//         blink(40000000);
//     }

//     return 0;
// }

// void nmi_handler(void)
// {    
//     while(1){
//         blink(1600000);
//     }
// }
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
