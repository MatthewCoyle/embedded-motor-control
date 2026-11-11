#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>

#define UART_LINE_CAPACITY 128u

typedef enum { UART_PENDING = 0, UART_LINE_READY, UART_LINE_OVERFLOW } uart_event_t;
typedef struct {
    char line[UART_LINE_CAPACITY];
    size_t length;
    bool overflow;
} uart_parser_t;

void uart_init(uart_parser_t *parser);
/* Feed one byte. On UART_LINE_READY, parser->line is zero-terminated.
 * Buffer is reset at next input byte, so consume the line immediately. */
uart_event_t uart_feed(uart_parser_t *parser, char byte);

#endif
