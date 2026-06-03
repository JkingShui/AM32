#ifndef UART_PRINT_H
#define UART_PRINT_H

#include <stdint.h>

void uart_print_init(uint32_t baudrate);
void uart_print_char(char c);
void uart_print_string(const char* str);
void uart_print_number(int32_t num);
void uart_print_hex(uint32_t num);

#endif