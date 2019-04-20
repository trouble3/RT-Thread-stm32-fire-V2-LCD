/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-08     obito0   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bsp_ili9341_lcd.h"

/* defined the LED0 pin: PC13 */
#define LED0_PIN    GET_PIN(B, 0)
#define LED1_PIN    GET_PIN(C, 4)
#define LED2_PIN    GET_PIN(C, 3)

int main(void)
{
    int count = 1;
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
		rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
		rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
		rt_pin_write(LED0_PIN, PIN_HIGH);
		rt_pin_write(LED1_PIN, PIN_HIGH);
		rt_pin_write(LED2_PIN, PIN_HIGH);
    while (count++)
    {
				rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
				rt_pin_write(LED1_PIN, PIN_LOW);
        rt_thread_mdelay(500);
        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED2_PIN, PIN_LOW);
        rt_thread_mdelay(500);
        rt_pin_write(LED2_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}

