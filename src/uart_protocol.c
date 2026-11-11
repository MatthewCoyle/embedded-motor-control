#include "uart_protocol.h"

void uart_init(uart_parser_t *p)
{
    *p = (uart_parser_t){0};
}

uart_event_t uart_feed(uart_parser_t *p, char byte)
{
    if (byte == '\n' || byte == '\r') {
        if (p->overflow) {
            p->overflow = false;
            p->length = 0u;
            p->line[0] = '\0';
            return UART_LINE_OVERFLOW;
        }
        if (p->length == 0u) return UART_PENDING;
        p->line[p->length] = '\0';
        p->length = 0u;
        return UART_LINE_READY;
    }
    if (p->overflow) return UART_PENDING;
    /* Reject the entire frame on binary/control data rather than interpreting
       a NUL-truncated prefix as a legitimate actuator command. */
    if (((unsigned char)byte < 32u && byte != '\t') || (unsigned char)byte > 126u) {
        p->overflow = true;
        return UART_PENDING;
    }
    if (p->length >= UART_LINE_CAPACITY - 1u) {
        p->overflow = true;
        return UART_PENDING;
    }
    p->line[p->length++] = byte;
    return UART_PENDING;
}
