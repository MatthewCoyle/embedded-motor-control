#include "console.h"
#include "uart_protocol.h"
#include <stdio.h>
#include <string.h>

static int run_stream(FILE *in, bool echo)
{
    uart_parser_t parser;
    motor_app_t app;
    uart_init(&parser);
    app_init(&app);
    int input;
    bool all_ok = true;
    puts("Embedded Motor Control | deterministic host simulation | type HELP");
    /* Read bytes, not strlen(fgets()): embedded NUL must reach the frame parser. */
    while ((input = fgetc(in)) != EOF) {
        uart_event_t event = uart_feed(&parser, (char)input);
        if (event == UART_LINE_READY) {
            if (parser.line[0] == '#' || parser.line[0] == '\0') continue;
            if (echo) printf("> %s\n", parser.line);
            if (!app_execute(&app, parser.line, stdout)) all_ok = false;
        } else if (event == UART_LINE_OVERFLOW) {
            fputs("ERR UART_LINE_OVERFLOW\n", stderr);
            all_ok = false;
        }
    }
    if (parser.length != 0u || parser.overflow) {
        uart_event_t event = uart_feed(&parser, '\n');
        if (event == UART_LINE_READY && parser.line[0] != '#') {
            if (echo) printf("> %s\n", parser.line);
            if (!app_execute(&app, parser.line, stdout)) all_ok = false;
        } else if (event == UART_LINE_OVERFLOW) {
            fputs("ERR UART_LINE_OVERFLOW\n", stderr);
            all_ok = false;
        }
    }
    if (ferror(in)) return 2;
    return all_ok ? 0 : 1;
}

int main(int argc, char **argv)
{
    if (argc == 1) {
        return run_stream(stdin, false);
    }
    if (argc == 3 && strcmp(argv[1], "--script") == 0) {
        FILE *file = fopen(argv[2], "r");
        if (file == NULL) { perror(argv[2]); return 2; }
        int exit_code = run_stream(file, true);
        fclose(file);
        return exit_code;
    }
    fprintf(stderr, "Usage: %s [--script examples/normal_operation.txt]\n", argv[0]);
    return 2;
}
