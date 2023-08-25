/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   change to new framework
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_gpio.h"
//#include "hci_interface.h"

/* defined the LED0 pin: PA5 */
#define LED0_PIN    GET_PIN(A, 5)

extern struct rt_spi_device *ble_spi;
SPI_HandleTypeDef hspi1;

int main(void)
{
//    HCI_TL_SPI_Init(NULL); // RT SPI 初始化
//    HCI_HAL_SPI_Init(NULL); //hal库SPI初始化
//    HCI_TL_SPI_Reset(); //管脚 reset
//
//    rt_uint8_t send_buf[4] = {0x01, 0x03, 0x0c, 0x00} ;
//    rt_size_t send_len = 4;
//    rt_size_t rev_len = 10;
//    rt_uint8_t rev_buf[10] = {0};

    //RTAPI直接发送
//    rt_pin_write(HCI_TL_SPI_CS_PIN, PIN_LOW);
////    if (RT_EOK != rt_spi_send_then_recv(ble_spi, &send_buf, send_len, &rev_buf,rev_len)){
////        rt_kprintf("Failed to send.");
////    }
//    HAL_SPI_TransmitReceive(&hspi1, send_buf, rev_buf, send_len, BUS_SPI1_POLL_TIMEOUT); //hal
//    rt_pin_write(HCI_TL_SPI_CS_PIN, PIN_HIGH); //disable cs finish
//

//    rt_kprintf("Recv：");
//    for (int i = 0; i < rev_len; ++i) {
//        rt_kprintf("%02x", rev_buf[i]);
//    }
//    rt_kprintf("\r\n");

    //    if (RT_EOK != HCI_TL_SPI_Send(send_buf, send_len)){
    //        rt_kprintf("Failed to send.");
    //    }
//    send_cmd(0x03, 0x003, 0, 0x00); //reset 发送
//
//    uint16_t size = 16;
//    uint8_t dataBuff[16];
//    for (int i = 0; i < 6; ++i) {
//        int dataLen = HCI_TL_SPI_Receive(dataBuff, size); // 接收
//        rt_kprintf("Recv size: %d buffer: %02x", dataLen, dataBuff[0]);
//        for (int i = 1; i < dataLen; ++i) {
//            rt_kprintf("%02x", dataBuff[i]);
//        }
//        rt_kprintf("\r\n");
//    }

    int count = 1;
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    while (count++)
    {
//        rt_pin_write(HCI_TL_SPI_CS_PIN, PIN_LOW);
//        if (RT_EOK != rt_spi_send_then_recv(ble_spi, &send_buf, send_len, &rev_buf,rev_len)){
//            rt_kprintf("Failed to send.");
//        }
//        HAL_SPI_TransmitReceive(&hspi1, buffer, read_char_buf, size, BUS_SPI1_POLL_TIMEOUT); //hal
//        rt_pin_write(HCI_TL_SPI_CS_PIN, PIN_HIGH); //disable cs finish
//        send_cmd(0x03, 0x003, 0, 0x00); //reset 发送
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}

//static int sample(void) {
//    rt_kprintf("sample 1 \r\n");
//    rt_thread_mdelay(2000);
//}
//
//MSH_CMD_EXPORT(sample, "sample");

