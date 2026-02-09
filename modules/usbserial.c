#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/crs.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/nvic.h>
#include "usbserial.h"

usbd_device *usbd_dev;

typedef uint8_t (*usbserial_process_cb_t)(uint8_t byte, void *context);

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
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
                                                               .bLength = USB_DT_ENDPOINT_SIZE,
                                                               .bDescriptorType = USB_DT_ENDPOINT,
                                                               .bEndpointAddress = 0x01,
                                                               .bmAttributes = USB_ENDPOINT_ATTR_BULK,
                                                               .wMaxPacketSize = 64,
                                                               .bInterval = 1,
                                                           },
                                                           {
                                                               .bLength = USB_DT_ENDPOINT_SIZE,
                                                               .bDescriptorType = USB_DT_ENDPOINT,
                                                               .bEndpointAddress = 0x82,
                                                               .bmAttributes = USB_ENDPOINT_ATTR_BULK,
                                                               .wMaxPacketSize = 64,
                                                               .bInterval = 1,
                                                           }};

static const struct
{
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
        .bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
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
    }};

static const struct usb_interface_descriptor comm_iface[] = {{.bLength = USB_DT_INTERFACE_SIZE,
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
                                                              .extralen = sizeof(cdcacm_functional_descriptors)}};

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
}};

static const struct usb_interface ifaces[] = {{
                                                  .num_altsetting = 1,
                                                  .altsetting = comm_iface,
                                              },
                                              {
                                                  .num_altsetting = 1,
                                                  .altsetting = data_iface,
                                              }};

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
    "Frannn oiiii",
    "Teste CDC-ACM",
    "4 8 15 16 23 42",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

volatile uint8_t cdc_connected = 0;

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev,
                                                             struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
                                                             void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    (void)complete;
    (void)buf;
    (void)usbd_dev;

    switch (req->bRequest)
    {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
    {
        /* Only flush RX when the host *clears* DTR (closing the port).
         * This preserves any bytes that arrived before the host toggled
         * DTR, avoiding lost characters when the host sends immediately
         * after enumeration. */
        if (req->wValue & 1)
        { /* DTR bit set */
            cdc_connected = 1;
        }
        else
        {
            cdc_connected = 0;
            usbserial_flush_rx();
        }
        return USBD_REQ_HANDLED;
    }
    case USB_CDC_REQ_SET_LINE_CODING:
        if (*len < sizeof(struct usb_cdc_line_coding))
        {
            return USBD_REQ_NOTSUPP;
        }

        return USBD_REQ_HANDLED;
    }

    return USBD_REQ_NOTSUPP;
}

#define disable_irq() __asm__("cpsid i")
#define enable_irq() __asm__("cpsie i")
#define set_msp(addr) __asm__("msr msp, %0" : : "r"(addr))

/* Reset STM32G4 clocks to default (HSI16, PLL off) */
void reset_clocks_to_default(void)
{
    /* 1. Switch SYSCLK to HSI16 */
    RCC_CR |= RCC_CR_HSION; // enable HSI16
    while (!(RCC_CR & RCC_CR_HSIRDY))
        ; // wait until ready

    RCC_CFGR &= ~RCC_CFGR_SW_MASK; // select HSI16 as SYSCLK
    while ((RCC_CFGR & RCC_CFGR_SWS_MASK) != 0x00)
        ; // check SYSCLK switch

    /* 2. Turn off PLL, HSE, HSI48 */
    RCC_CR &= ~RCC_CR_PLLON;
    RCC_CR &= ~RCC_CR_HSEON;
    RCC_CRRCR &= ~RCC_CRRCR_HSI48ON;

    /* 3. Reset prescalers */
    RCC_CFGR &= ~RCC_CFGR_HPRE_MASK;  // HCLK = SYSCLK
    RCC_CFGR &= ~RCC_CFGR_PPRE1_MASK; // PCLK1 = HCLK
    RCC_CFGR &= ~RCC_CFGR_PPRE2_MASK; // PCLK2 = HCLK

    /* 4. Disable USB & CRS clocks */
    RCC_AHB2ENR &= ~RCC_APB1ENR1_USBEN;
    RCC_APB1ENR1 &= ~RCC_APB1ENR1_CRSEN;

    /* 5. Optional: reset RCC interrupt registers */
    RCC_CIER = 0x00000000;

    /* 6. Reset GPIOs if needed */
    // Example: set USB pins to input to avoid glitches
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO11 | GPIO12);
}

volatile uint32_t rxi = 0, rxo = 0;
volatile uint32_t txi = 0, txo = 0;

uint8_t rxbuff[USB_BUFF_SIZE];
uint8_t txbuff[USB_BUFF_SIZE];

volatile uint32_t meas_flag = 0;
volatile uint32_t freq_flag = 0;

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    if (!cdc_connected)
    {
        // Drain and discard
        char tmp[64];
        usbd_ep_read_packet(usbd_dev, 0x01, tmp, sizeof(tmp));
        return;
    }

    char buf[64];
    uint32_t len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    for (uint32_t i = 0; i < len; i++)
    {
        uint32_t next = (rxi + 1) % USB_BUFF_SIZE;
        if (next != rxo)
        {
            rxbuff[rxi] = buf[i];
            rxi = next;
        }
    }
}

volatile uint8_t tx_busy = 0, tx_zlp = 0;

static void cdcacm_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    disable_irq();
    if (txi == txo)
    {
        if (tx_zlp)
        {
            usbd_ep_write_packet(usbd_dev, 0x82, NULL, 0);
            tx_zlp = 0;
        }
        else
        {
            tx_busy = 0; // TX idle
        }
        return;
    }

    uint32_t len = (txo > txi) ? (txo - txi) : (USB_BUFF_SIZE - txi);
    if (len > 64)
        len = 64;

    uint32_t sent = usbd_ep_write_packet(usbd_dev, 0x82, &txbuff[txi], len);

    if (sent)
    {
        txi = (txi + sent) % USB_BUFF_SIZE;
        tx_busy = 1;
        if ((txi == txo) && (sent == 64))
        {
            tx_zlp = 1;
        }
    }
    enable_irq();
}

void usbserial_send_tx(uint8_t *data, uint32_t len)
{
    uint32_t i = 0;

    while (i < len)
    {
        disable_irq();
        uint32_t next = (txo + 1) % USB_BUFF_SIZE;

        if (next != txi)
        {
            txbuff[txo] = data[i++];
            txo = next;
        }

        /* Kick TX if idle */
        if (!tx_busy)
        {
            tx_busy = 1;
            cdcacm_data_tx_cb(usbd_dev, 0x82);
        }
        enable_irq();
    }
}

/**
 * Read data from the USB receive buffer
 * - data Pointer to buffer where data will be copied
 * - max_len Maximum number of bytes to read
 * Returns number of bytes actually read (0 if buffer empty)
 */
uint32_t usbserial_read_rx(uint8_t *data, uint32_t max_len)
{
    uint32_t bytes_available = 0;
    uint32_t bytes_to_read = 0;

    // Disable interrupts while accessing shared buffer indices
    disable_irq();

    // Calculate available bytes (handle wrap-around)
    if (rxi >= rxo)
    {
        bytes_available = rxi - rxo;
    }
    else
    {
        bytes_available = USB_BUFF_SIZE - rxo + rxi;
    }

    // Limit to requested length
    bytes_to_read = (bytes_available < max_len) ? bytes_available : max_len;

    // Read data from buffer
    for (uint32_t i = 0; i < bytes_to_read; i++)
    {
        data[i] = rxbuff[rxo];
        rxo = (rxo + 1) % USB_BUFF_SIZE;
    }

    // Re-enable interrupts
    enable_irq();

    return bytes_to_read;
}

/**
 * Check how many bytes are available in receive buffer
 * Returns number of bytes available for reading
 */
uint32_t usbserial_rx_available(void)
{
    uint32_t available;

    disable_irq();
    if (rxi >= rxo)
    {
        available = rxi - rxo;
    }
    else
    {
        available = USB_BUFF_SIZE - rxo + rxi;
    }
    enable_irq();

    return available;
}

/**
 * Read a single byte from buffer (blocking with timeout)
 * - data Pointer to store the byte
 * - timeout_ms Maximum time to wait (0 = infinite)
 * Returns 1 if byte read, 0 if timeout
 */
uint8_t usbserial_read_byte(uint8_t *data, uint32_t timeout_ms)
{
    uint32_t start_time = clock_ticks; // Assuming you have a millisecond timer

    while (1)
    {
        disable_irq();
        if (rxi != rxo)
        { // Buffer not empty
            *data = rxbuff[rxo];
            rxo = (rxo + 1) % USB_BUFF_SIZE;
            enable_irq();
            return 1;
        }
        enable_irq();

        // Check timeout
        if (timeout_ms > 0)
        {
            uint32_t elapsed = clock_ticks - start_time;
            if (elapsed >= timeout_ms)
            {
                return 0; // Timeout
            }
        }

        // Optional: Enter low-power mode or yield
        // __WFI();  // Wait for interrupt
    }
}

/**
 * Read until a specific character is found
 * - buffer Destination buffer
 * - max_len Maximum bytes to read
 * - terminator Terminator character (e.g., '\n')
 * - timeout_ms Maximum time to wait
 * Returns number of bytes read (including terminator if found)
 */
uint32_t usbserial_read_until(uint8_t *buffer, uint32_t max_len,
                              uint8_t terminator, uint32_t timeout_ms)
{
    uint32_t bytes_read = 0;
    // uint32_t start_time = clock_ticks;
    uint8_t ch;

    while (bytes_read < max_len)
    {
        if (usbserial_read_byte(&ch, timeout_ms) == 0)
        {
            break; // Timeout
        }

        buffer[bytes_read++] = ch;

        if (ch == terminator)
        {
            break; // Found terminator
        }

        // Reset timeout for next byte
        // start_time = clock_ticks;
    }

    return bytes_read;
}

/**
 * Peek at next byte without removing it from buffer
 * Returns next byte, or -1 if buffer empty
 */
int16_t usbserial_peek(void)
{
    int16_t result = -1;

    disable_irq();
    if (rxi != rxo)
    {
        result = rxbuff[rxo];
    }
    enable_irq();

    return result;
}

/**
 * Clear/Flush the receive buffer
 */
void usbserial_flush_rx(void)
{
    disable_irq();
    rxo = rxi; // Reset indices (buffer is empty)
    enable_irq();
}

// /**
//  * Advanced: Read with callback for processing
//  * - process_callback Function to process each byte
//  * - context User context pointer
//  * Returns number of bytes processed
//  */
// uint32_t usbserial_process_rx(usbserial_process_cb_t process_callback, void* context)
// {
//     uint32_t bytes_processed = 0;
//     uint8_t ch;

//     while (usbserial_rx_available() > 0) {
//         disable_irq();
//         ch = rxbuff[rxo];
//         rxo = (rxo + 1) % USB_BUFF_SIZE;
//         enable_irq();

//         // Call user callback
//         if (process_callback(ch, context) == 0) {
//             break;  // Callback asked to stop
//         }

//         bytes_processed++;
//     }

//     return bytes_processed;
// }

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_tx_cb);
    usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

    usbd_register_control_callback(
        usbd_dev,
        USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        cdcacm_control_request);
}

void usbserial_init()
{
    // Enable HSI48
    RCC_CRRCR |= RCC_CRRCR_HSI48ON;
    // Wait until ready
    while (!(RCC_CRRCR & RCC_CRRCR_HSI48RDY))
        ; // enable HSI48
    rcc_set_clock48_source(RCC_CCIPR_CLK48SEL_HSI48);

    rcc_periph_clock_enable(RCC_USB);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_CRS);
    crs_autotrim_usb_enable();

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12); // USB_FS

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
    nvic_enable_irq(NVIC_USB_LP_IRQ);
}

void usb_hp_isr(void)
{
}

void usb_lp_isr(void)
{
    usbd_poll(usbd_dev);
}
/**
 * Disconnect and reset USB peripheral completely
 *
 * This function safely disables USB, resets the peripheral,
 * and puts pins in a safe state. Call usbserial_init() to restart.
 */
void usbserial_disconnect(void)
{
    // 1. Disable USB interrupts
    nvic_disable_irq(NVIC_USB_HP_IRQ);
    nvic_disable_irq(NVIC_USB_LP_IRQ);
    nvic_disable_irq(NVIC_USB_WAKEUP_IRQ);

    // 2. Reset connection flag and buffer indices
    disable_irq();
    cdc_connected = 0;
    rxi = rxo = 0;
    txi = txo = 0;
    tx_busy = 0;
    tx_zlp = 0;
    enable_irq();

    // 3. Deinit USB peripheral if device exists
    if (usbd_dev != NULL)
    {
        // Disconnect USB from host (pull-up disable)
        // Note: For STM32G4, USB_FS pull-up is internal on PA12 (DP line)
        // We'll reset the peripheral which effectively disconnects

        // Force USB peripheral reset
        RCC_APB1RSTR1 |= RCC_APB1RSTR1_USBRST;
        __asm__("nop");
        __asm__("nop");
        RCC_APB1RSTR1 &= ~RCC_APB1RSTR1_USBRST;

        // Clear USB device pointer
        usbd_dev = NULL;
    }

    // 4. Disable USB peripheral clock
    RCC_APB1ENR1 &= ~RCC_APB1ENR1_USBEN;

    // 6. Disable CRS (Clock Recovery System) if it was enabled
    rcc_periph_clock_disable(RCC_CRS);

    // 7. Reset USB clock source (HSI48)
    RCC_CRRCR &= ~RCC_CRRCR_HSI48ON;

    // 8. Clear any pending USB interrupts
    NVIC_ICPR(NVIC_USB_HP_IRQ) = 1;
    NVIC_ICPR(NVIC_USB_LP_IRQ) = 1;
    NVIC_ICPR(NVIC_USB_WAKEUP_IRQ) = 1;

    // 9. Optional: Clear USB_SRAM (USB dedicated RAM)
    // STM32G4 has 1KB USB dedicated RAM at 0x40006000-0x400063FF
    // Might be good to clear to avoid stale data
    volatile uint32_t *usb_sram = (volatile uint32_t *)0x40006000;
    for (int i = 0; i < 256; i++)
    { // 256 * 4 bytes = 1KB
        usb_sram[i] = 0x00000000;
    }

    // 10. Clear USB control buffer
    memset(usbd_control_buffer, 0, sizeof(usbd_control_buffer));

    // 11. Reset USB buffer indices in software
    memset(rxbuff, 0, USB_BUFF_SIZE);
    memset(txbuff, 0, USB_BUFF_SIZE);
}

/**
 * Check if USB is currently connected and functional
 * Returns 1 if USB is initialized and connected, 0 otherwise
 */
uint8_t usbserial_is_connected(void)
{
    return (usbd_dev != NULL) && cdc_connected;
}

/**
 * Get USB device status
 * Returns Pointer to USB device if initialized, NULL otherwise
 */
usbd_device *usbserial_get_device(void)
{
    return usbd_dev;
}