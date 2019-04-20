/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-24     yangjie      the first version
 */

/*
 * 程序清单：创建、初始化/脱离线程
 *
 * 这个例子会创建两个线程，一个动态线程，一个静态线程。
 * 静态线程在运行完毕后自动被系统脱离，动态线程一直打印计数。
 */
#include <rtthread.h>
#include "bsp_ili9341_lcd.h"

#define THREAD_PRIORITY         25
#define THREAD_STACK_SIZE       512
#define THREAD_TIMESLICE        5

static rt_thread_t tid1 = RT_NULL;


extern void Lcd_Delay( rt_uint32_t nCount);


/* 线程1的入口函数 */
static void thread1_entry(void *parameter)
{
		rt_uint32_t n;

		MX_GPIO_Init();
		MX_FSMC_Init();
		
    while (1)
    {
			LCD_Init();	

			LCD_Clear(0, 0, 240, 320, BACKGROUND);	

			LCD_DispChar(60, 60, 'A', RED);


			LCD_DispStr(10, 10, (uint8_t *)"This is a lcd demo to display ascii", RED);	

			LCD_DispStr(40, 100, (uint8_t *)"count:", RED);

			for( n=0; n<500000; n++ )
			{
				LCD_DisNum(100, 100, n, RED);
				Lcd_Delay(0xAFFf>>4);
			}
    }
}


/* 线程示例 */
int thread_sample(void)
{
    /* 创建线程1，名称是thread1，入口是thread1_entry*/
    tid1 = rt_thread_create("thread1",
                            thread1_entry, RT_NULL,
                            THREAD_STACK_SIZE,
                            THREAD_PRIORITY, THREAD_TIMESLICE);

    /* 如果获得线程控制块，启动这个线程 */
    if (tid1 != RT_NULL)
        rt_thread_startup(tid1);

    return 0;
}

/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(thread_sample, thread sample);
