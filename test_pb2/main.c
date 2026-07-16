
#include <stdint.h>

#define PERIPH_BASE         ((uint32_t)0x40000000)
#define AHBPERIPH1_BASE     (PERIPH_BASE + 0x00020000)
#define AHBPERIPH2_BASE     (PERIPH_BASE + 0x08000000)

#define CRM_BASE            (AHBPERIPH1_BASE + 0x00001000)
#define GPIOA_BASE          (AHBPERIPH2_BASE + 0x00000000)
#define GPIOB_BASE          (AHBPERIPH2_BASE + 0x00000400)

#define CRM_CTRL            (*(volatile uint32_t *)(CRM_BASE + 0x00))
#define CRM_CFG             (*(volatile uint32_t *)(CRM_BASE + 0x04))
#define CRM_AHBENR          (*(volatile uint32_t *)(CRM_BASE + 0x14))

#define GPIOA_CFGR          (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_OMODE         (*(volatile uint32_t *)(GPIOA_BASE + 0x04))
#define GPIOA_ODRVR         (*(volatile uint32_t *)(GPIOA_BASE + 0x08))
#define GPIOA_PULL          (*(volatile uint32_t *)(GPIOA_BASE + 0x0C))
#define GPIOA_ODT           (*(volatile uint32_t *)(GPIOA_BASE + 0x14))
#define GPIOA_SCR           (*(volatile uint32_t *)(GPIOA_BASE + 0x18))

#define GPIOB_CFGR          (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_OMODE         (*(volatile uint32_t *)(GPIOB_BASE + 0x04))
#define GPIOB_ODRVR         (*(volatile uint32_t *)(GPIOB_BASE + 0x08))
#define GPIOB_PULL          (*(volatile uint32_t *)(GPIOB_BASE + 0x0C))
#define GPIOB_ODT           (*(volatile uint32_t *)(GPIOB_BASE + 0x14))
#define GPIOB_SCR           (*(volatile uint32_t *)(GPIOB_BASE + 0x18))

#define GPIO_PINS_2         ((uint16_t)0x0004)
#define GPIO_PINS_15        ((uint16_t)0x8000)

static void delay_ms(volatile uint32_t ms)
{
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 2000; j++);
}

static void system_clock_config(void)
{
    volatile uint32_t wait_count;

    CRM_CTRL |= (1 << 0);
    wait_count = 0;
    while(!(CRM_CTRL & (1 << 1)) && wait_count < 100000) wait_count++;

    CRM_CFG &= ~((uint32_t)0x03 << 0);
    wait_count = 0;
    while(((CRM_CFG >> 2) & 0x03) != 0 && wait_count < 100000) wait_count++;
}

int main(void)
{
    system_clock_config();

    CRM_AHBENR |= (1 << 17);
    CRM_AHBENR |= (1 << 18);

    GPIOA_CFGR &= ~((uint32_t)0x03 << (15 * 2));
    GPIOA_CFGR |= ((uint32_t)0x01 << (15 * 2));
    GPIOA_OMODE &= ~((uint32_t)0x01 << 15);
    GPIOA_ODRVR &= ~((uint32_t)0x03 << (15 * 2));
    GPIOA_ODRVR |= ((uint32_t)0x01 << (15 * 2));
    GPIOA_PULL &= ~((uint32_t)0x03 << (15 * 2));
    GPIOA_SCR = (uint32_t)GPIO_PINS_15;

    GPIOB_CFGR &= ~((uint32_t)0x03 << (2 * 2));
    GPIOB_CFGR |= ((uint32_t)0x01 << (2 * 2));
    GPIOB_OMODE &= ~((uint32_t)0x01 << 2);
    GPIOB_ODRVR &= ~((uint32_t)0x03 << (2 * 2));
    GPIOB_ODRVR |= ((uint32_t)0x01 << (2 * 2));
    GPIOB_PULL &= ~((uint32_t)0x03 << (2 * 2));

    while(1)
    {
        GPIOB_SCR = ((uint32_t)GPIO_PINS_2 << 16);
        delay_ms(500);
        GPIOB_SCR = (uint32_t)GPIO_PINS_2;
        delay_ms(500);
    }
}

