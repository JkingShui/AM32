#include "uart_print.h"
#include "at32f421_crm.h"
#include "at32f421_gpio.h"
#include "at32f421_usart.h"

void uart_print_init(uint32_t baudrate)
{
    gpio_init_type gpio_init_struct;

    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    // PB6 是 USART1_TX 的默认引脚，PB7 是 RX
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_6;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_0);

    usart_init(USART1, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART1, TRUE);
    usart_enable(USART1, TRUE);
}

void uart_print_char(char c)
{
    // 等待发送数据寄存器为空
    while (!(USART1->sts & USART_TDBE_FLAG));
    USART1->dt = (uint8_t)c;
}

void uart_print_string(const char* str)
{
    while (*str) {
        uart_print_char(*str++);
    }
}

void uart_print_number(int32_t num)
{
    char buffer[12];
    uint8_t i = 0;

    if (num < 0) {
        uart_print_char('-');
        num = -num;
    }

    if (num == 0) {
        uart_print_char('0');
        return;
    }

    while (num > 0 && i < sizeof(buffer)) {
        buffer[i++] = '0' + (num % 10);
        num /= 10;
    }

    while (i > 0) {
        uart_print_char(buffer[--i]);
    }
}

void uart_print_hex(uint32_t num)
{
    const char hex_chars[] = "0123456789ABCDEF";
    char buffer[9];
    uint8_t i = 0;

    if (num == 0) {
        uart_print_string("0x0");
        return;
    }

    uart_print_string("0x");

    while (num > 0 && i < 8) {
        buffer[i++] = hex_chars[num & 0xF];
        num >>= 4;
    }

    while (i > 0) {
        uart_print_char(buffer[--i]);
    }
}