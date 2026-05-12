#include "common.h"

key_t key_menu;
key_t key_enter;



void key_init(void)
{
    key_menu.port  = GPIOA;
    key_menu.pin   = GPIO_PIN_4;
    key_menu.last_level = 1;   // 初始认为是松开（高电平）

    key_enter.port = GPIOC;
    key_enter.pin  = GPIO_PIN_3;
    key_enter.last_level = 1;
}


key_event_t key_update(key_t *k)
{
    uint8_t cur = HAL_GPIO_ReadPin(k->port, k->pin);

    key_event_t ev = KEY_EVENT_NONE;

    if (k->last_level == 1 && cur == 0)   // 下降沿 = 刚按下
    {
        ev = KEY_EVENT_SHORT;
    }

    k->last_level = cur;
    return ev;
}
