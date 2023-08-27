/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-25     YTR       the first version
 */
#ifndef APPLICATIONS_HCI_INTERFACE_C_
#define APPLICATIONS_HCI_INTERFACE_C_

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_gpio.h"
#include "rtthread_driver_spi.h"

#include "drivers/hci_driver.h"
#include "drivers/hci_h4.h"

#include "logging/bt_log_impl.h"
#include "common/bt_buf.h"

struct rt_spi_device *ble_spi = RT_NULL;
static uint32_t SPI1InitCounter = 0;

/* Private function prototypes -----------------------------------------------*/
int32_t IsDataAvailable(void);

int32_t HCI_TL_SPI_Init(void* pConf)
{
    /******PIN******/
    rt_pin_mode(HCI_TL_SPI_IRQ_PIN, PIN_MODE_INPUT); // IRQ input
    rt_pin_mode(HCI_TL_SPI_CS_PIN, PIN_MODE_OUTPUT); // cs output

    /*******SPI******/
    ble_spi = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));
    if(RT_NULL == ble_spi)
    {
        printk("Failed to malloc the spi device.");
        return -RT_ENOMEM;
    }
    if (RT_EOK != rt_spi_bus_attach_device(ble_spi, "spi10", "spi1", RT_NULL))
    {
        printk("Failed to attach the spi device.");
        return -RT_ERROR;
    }

    /*******spi device config*******/
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_1 | RT_SPI_MSB;  // CPOL 0 CPHA 1
    cfg.max_hz =  1 * 1000 * 1000;                           /* 1M */

    if (RT_EOK != rt_spi_configure(ble_spi, &cfg)) {
        printk("Failed to config the spi device.");
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief  Reads from BlueNRG SPI buffer and store data into local buffer.
 *
 * @param  buffer : Buffer where data from SPI are stored
 * @param  size   : Buffer size
 * @retval int32_t: Number of read bytes
 */
int32_t HCI_TL_SPI_Receive(uint8_t* buffer, uint16_t size)
{
  uint16_t byte_count;
  uint8_t len = 0;
  uint8_t char_00 = 0x00;
  volatile uint8_t read_char;

  uint8_t header_master[HEADER_SIZE] = {0x0b, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];

  /* CS reset */
  rt_pin_write(HCI_TL_SPI_CS_PIN, PIN_LOW);

  /* Read the header */
  rt_spi_transfer(ble_spi, &header_master, &header_slave, HEADER_SIZE); //RTAPI

  /* device is ready */
  byte_count = (header_slave[4] << 8)| header_slave[3];

  if(byte_count > 0)
  {
    /* avoid to read more data than the size of the buffer */
    if (byte_count > size)
    {
      byte_count = size;
    }

    for(len = 0; len < byte_count; len++)
    {
      rt_spi_transfer(ble_spi, &char_00, (uint8_t*)&read_char, 1);
      buffer[len] = read_char;
    }
  }

  /**
   * To be aligned to the SPI protocol.
   * Can bring to a delay inside the frame, due to the BlueNRG-2 that needs
   * to check if the header is received or not.
   */
  uint32_t tickstart = rt_tick_get();
  while ((rt_tick_get() - tickstart) < TIMEOUT_IRQ_HIGH) {
    if (rt_pin_read(HCI_TL_SPI_IRQ_PIN) == PIN_LOW) {
      break;
    }
  }

  /* Release CS line */
  rt_pin_write(HCI_TL_SPI_CS_PIN, PIN_HIGH);

  return len;
}

/**
 * @brief  Writes data from local buffer to SPI.
 *
 * @param  buffer : data buffer to be written
 * @param  size   : size of first data buffer to be written
 * @retval int32_t: Number of read bytes
 */
int32_t HCI_TL_SPI_Send(uint8_t* buffer, uint16_t size)
{
  int32_t result;
  uint16_t rx_bytes;

  uint8_t header_master[HEADER_SIZE] = {0x0a, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];

  static uint8_t read_char_buf[MAX_BUFFER_SIZE];
  uint32_t tickstart = rt_tick_get();


  do
  {
    uint32_t tickstart_data_available = rt_tick_get();

    result = 0;

    /* CS reset */
    rt_pin_write(HCI_TL_SPI_CS_PIN, PIN_LOW);

    /*
     * Wait until BlueNRG-2 is ready.
     * When ready it will raise the IRQ pin.
     */
    while(!IsDataAvailable())
    {
      if((rt_tick_get() - tickstart_data_available) > TIMEOUT_DURATION)
      {
        printk("%d timeout\r\n", rt_tick_get() - tickstart_data_available);
        result = -3;
        break;
      }
    }
    if(result == -3)
    {
      /* The break causes the exiting from the "while", so the CS line must be released */
      rt_pin_write(HCI_TL_SPI_CS_PIN, PIN_HIGH);
      break;
    }

    /* Read header */
    rt_spi_transfer(ble_spi, &header_master, &header_slave, HEADER_SIZE);

    printk("Send header size: %d buffer: %02x", HEADER_SIZE, header_slave[0]);
    for (int i = 1; i < HEADER_SIZE; ++i) {
        printk("%02x", header_slave[i]);
    }
    printk("\r\n");

    rx_bytes = (((uint16_t)header_slave[2])<<8) | ((uint16_t)header_slave[1]);

    if(rx_bytes >= size)
    {
      /* Buffer is big enough */
      rt_spi_transfer(ble_spi, buffer, &read_char_buf, size);
    }
    else
    {
      /* Buffer is too small */
      result = -2;
    }

    /* Release CS line */
    rt_pin_write(HCI_TL_SPI_CS_PIN, PIN_HIGH);

    if((rt_tick_get() - tickstart) > TIMEOUT_DURATION)
    {
      result = -3;
      break;
    }
  } while(result < 0);

  /**
   * To be aligned to the SPI protocol.
   * Can bring to a delay inside the frame, due to the BlueNRG-2 that needs
   * to check if the header is received or not.
   */
  tickstart = rt_tick_get();
  while ((rt_tick_get() - tickstart) < TIMEOUT_IRQ_HIGH) {
    if (rt_pin_read(HCI_TL_SPI_IRQ_PIN) == PIN_LOW) {
      break;
    }
  }

  return result;
}


/**
 * @brief  Reports if the BlueNRG has data for the host micro.
 *
 * @param  None
 * @retval int32_t: 1 if data are present, 0 otherwise
 */
int32_t IsDataAvailable(void)
{
  return (rt_pin_read(HCI_TL_SPI_IRQ_PIN) == PIN_HIGH);
}

static int hci_driver_open(void)
{
    HCI_TL_SPI_Init(NULL); // RT SPI 初始化

    printk("hci_driver_open, SPI_config\n");

    return 0;
}

int switch_net_buf_type(uint8_t type)
{
    switch (type)
    {
    case BT_BUF_ACL_OUT:
        return HCI_ACLDATA_PKT;
    case BT_BUF_CMD:
        return HCI_COMMAND_PKT;
    default:
        printk("Unknown buffer type");
    }
    return 0;
}
static int hci_driver_send(struct net_buf *buf)
{
    uint8_t type = bt_buf_get_type(buf);

//    net_buf_push_u8(buf, type); //前面没有空间了 插入不了了
//    uint8_t* data = buf->data;

    uint8_t len = buf->len;

    if(len >= MAX_BUFFER_SIZE)
    {
        return -1;
    }

    uint8_t data[MAX_BUFFER_SIZE];
    data[0] = switch_net_buf_type(type);
    memcpy(data + 1, buf->data, len); //data[0] is the type  copy to data[1]

    printk("hci_driver_send, type: %d, len: %d, data: %02x:%02x:%02x:%02x:%02x:%02x\n", type, len, data[0], data[1], data[2], data[3], data[4], data[5]);

    if (HCI_TL_SPI_Send(data, len + 1) < 0) { // type +1
        return -1;
    }
    net_buf_unref(buf);
    return 0;
}

static int hci_driver_h4_recv(uint8_t *buf, uint16_t len)
{
    return HCI_TL_SPI_Receive(buf, len);
}

void hci_driver_init_loop(void)
{
    uint8_t data[MAX_BUFFER_SIZE];
    uint8_t len = MAX_BUFFER_SIZE;
    int ret = HCI_TL_SPI_Receive(data, len); //ret 接收长度
    if(ret > 0 && (data[0] != 0)) // type cant be 0
    {
        printk("hci_driver_init_loop, ret: %d, data: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n", ret, data[0], data[1], data[2], data[3], data[4], data[5],data[6],data[7]);

        struct net_buf *buf;
        switch(data[0])
        {
            case HCI_EVENT_PKT:
                buf = bt_buf_get_controller_tx_evt();
                break;
            case HCI_ACLDATA_PKT:
                buf = bt_buf_get_controller_tx_acl();
                break;
            default:
                return;
        }

        if (buf)
        {
            net_buf_add_mem(buf, data + 1, ret - 1);
            bt_recv(buf);
        }
    }
}

static const struct bt_hci_driver drv = {
        .open = hci_driver_open,
        .send = hci_driver_send,
};


void hci_driver_init(void)
{
    bt_hci_driver_register(&drv);
}


#endif /* APPLICATIONS_HCI_INTERFACE_C_ */
