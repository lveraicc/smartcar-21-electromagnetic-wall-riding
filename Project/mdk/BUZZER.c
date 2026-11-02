#include "BUZZER.h"

static volatile uint8 buzzer_enable = 0;

void buzzer_init(void)
{
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    buzzer_enable = 0;
}

void buzzer_on(void)
{
    buzzer_enable = 1;
    gpio_set_level(BUZZER_PIN, GPIO_HIGH);
}

void buzzer_off(void)
{
    buzzer_enable = 0;
    gpio_set_level(BUZZER_PIN, GPIO_LOW);
}

void buzzer_set_enable(uint8 enable)
{
    if(enable)
    {
        buzzer_on();
    }
    else
    {
        buzzer_off();
    }
}

uint8 buzzer_get_enable(void)
{
    return buzzer_enable;
}
