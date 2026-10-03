#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef enum { BURST_SERIAL_USB, BURST_SERIAL_UART, BURST_SERIAL_COUNT } burst_serial_port_t;
#define BURST_SERIAL_LINE_CAPACITY 384

void burst_serial_init(void);
int burst_serial_poll_line(char *line, size_t capacity);
burst_serial_port_t burst_serial_port(void);
unsigned burst_serial_baud(void);
/* Human chat accepts CR as Enter and allows pauses while typing. Raw binary
 * commands retain their existing LF/CRLF framing and three-second timeout. */
void burst_serial_text_mode(bool enabled);
bool burst_serial_receive(void *data, size_t size, unsigned timeout_ms);
bool burst_serial_send(const void *data, size_t size);
