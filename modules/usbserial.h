#ifndef USBSERIAL_H_FILE
#define USBSERIAL_H_FILE

#include <stdint.h>
#include <libopencm3/usb/usbd.h>

extern volatile uint32_t clock_ticks;
extern volatile uint32_t freqw;

// Buffer size for USB serial
#ifndef BUFF_SIZE
#define BUFF_SIZE 1024
#endif

// Function pointer type for callback processing
typedef uint8_t (*usbserial_process_cb_t)(uint8_t byte, void* context);

// Initialization
void usbserial_init(void);

// Clock management
void reset_clocks_to_default(void);

// Data transmission
void usbserial_send_tx(uint8_t* data, uint32_t len);

// Data reception functions
uint32_t usbserial_read_rx(uint8_t* data, uint32_t max_len);
uint32_t usbserial_rx_available(void);
uint8_t usbserial_read_byte(uint8_t* data, uint32_t timeout_ms);
uint32_t usbserial_read_until(uint8_t* buffer, uint32_t max_len, 
                              uint8_t terminator, uint32_t timeout_ms);
int16_t usbserial_peek(void);
void usbserial_flush_rx(void);
uint32_t usbserial_process_rx(usbserial_process_cb_t process_callback, void* context);
// uint32_t usbserial_read_line(char* line_buffer, uint32_t max_len, uint8_t echo);

#endif