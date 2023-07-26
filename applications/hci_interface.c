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
#include "hci_interface.h"
#include "stm32l4xx_hal.h"

struct rt_spi_device *ble_spi = RT_NULL;
EXTI_HandleTypeDef hexti0;
SPI_HandleTypeDef hspi1;
static uint32_t SPI1InitCounter = 0;

/* Private function prototypes -----------------------------------------------*/
static void HCI_TL_SPI_Enable_IRQ(void);
static void HCI_TL_SPI_Disable_IRQ(void);
int32_t IsDataAvailable(void);
int32_t BSP_SPI1_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t Length);

/******************** IO Operation and BUS services ***************************/
/**
 * @brief  Enable SPI IRQ.
 * @param  None
 * @retval None
 */
static void HCI_TL_SPI_Enable_IRQ(void)
{
  //HAL_NVIC_EnableIRQ(HCI_TL_SPI_EXTI_IRQn);
  rt_pin_irq_enable(HCI_TL_SPI_EXTI_PIN, PIN_IRQ_ENABLE);
}

/**
 * @brief  Disable SPI IRQ.
 * @param  None
 * @retval None
 */
static void HCI_TL_SPI_Disable_IRQ(void)
{
  //HAL_NVIC_DisableIRQ(HCI_TL_SPI_EXTI_IRQn);
  rt_pin_irq_enable(HCI_TL_SPI_EXTI_PIN, PIN_IRQ_DISABLE);
}

/**
 * @brief  Initializes the peripherals communication with the BlueNRG
 *         Expansion Board (via SPI, I2C, USART, ...)
 *
 * @param  void* Pointer to configuration struct
 * @retval int32_t Status
 */
void HCI_TL_IRQ(void *args)
{
    rt_kprintf("IRQ!\n");
}

int32_t HCI_TL_SPI_Init(void* pConf)
{
    /******PIN******/
    rt_pin_mode(HCI_TL_SPI_EXTI_PIN, PIN_MODE_INPUT); // IRQ input
    rt_pin_mode(HCI_TL_RST_PIN, PIN_MODE_OUTPUT); // reset output
    rt_pin_mode(HCI_TL_SPI_CS_PIN, PIN_MODE_OUTPUT); // cs output
    if (RT_EOK != rt_pin_attach_irq(HCI_TL_SPI_EXTI_PIN,  PIN_IRQ_MODE_RISING, HCI_TL_IRQ, RT_NULL)) // IRQFUNC
    {
        rt_kprintf("Failed to attach irq.");
        return -RT_ERROR;
    }

    /*******SPI******/
    ble_spi = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));
    if(RT_NULL == ble_spi)
    {
        rt_kprintf("Failed to malloc the spi device.");
        return -RT_ENOMEM;
    }
    if (RT_EOK != rt_spi_bus_attach_device(ble_spi, "spi10", "spi1", RT_NULL))
    {
        rt_kprintf("Failed to attach the spi device.");
        return -RT_ERROR;
    }

    /* spi…Ë±∏≈‰÷√  */
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_1 | RT_SPI_MSB;  // CPOL 0 CPHA 1
    cfg.max_hz =  1 * 1000 * 1000;                           /* 1M */

    if (RT_EOK != rt_spi_configure(ble_spi, &cfg)) {
        rt_kprintf("Failed to config the spi device.");
        return -RT_ERROR;
    }
    return RT_EOK;
}

/**
 * @brief Reset BlueNRG module.
 *
 * @param  None
 * @retval int32_t 0
 */
int32_t HCI_TL_SPI_Reset(void)
{
  // Deselect CS PIN for BlueNRG to avoid spurious commands
  rt_pin_write(HCI_TL_SPI_CS_PIN, PIN_HIGH);

  rt_pin_write(HCI_TL_RST_PIN, PIN_LOW);
  rt_thread_mdelay(5);

  rt_pin_write(HCI_TL_RST_PIN, PIN_HIGH);
  rt_thread_mdelay(5);
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

//  HCI_TL_SPI_Disable_IRQ();

  /* CS reset */
  rt_pin_write(HCI_TL_SPI_CS_PIN, PIN_LOW);

  /* Read the header */
//  BSP_SPI1_SendRecv(header_master, header_slave, HEADER_SIZE);
//  rt_spi_send_then_recv(ble_spi, &header_master, HEADER_SIZE, &header_slave, HEADER_SIZE);
  HAL_SPI_TransmitReceive(&hspi1, header_master, header_slave, HEADER_SIZE, BUS_SPI1_POLL_TIMEOUT);

  rt_kprintf("Recv header size: %d buffer: %02x", HEADER_SIZE, header_slave[0]);
  for (int i = 1; i < HEADER_SIZE; ++i) {
      rt_kprintf("%02x", header_slave[i]);
  }
  rt_kprintf("\r\n");

  /* device is ready */
  byte_count = (header_slave[4] << 8)| header_slave[3];
//  byte_count = (0x0 << 8)| 0x7;
//  rt_kprintf("byte_count %d \r\n", byte_count);

  if(byte_count > 0)
  {

    /* avoid to read more data than the size of the buffer */
    if (byte_count > size)
    {
      byte_count = size;
    }

    for(len = 0; len < byte_count; len++)
    {
//      BSP_SPI1_SendRecv(&char_00, (uint8_t*)&read_char, 1);
//      rt_spi_send_then_recv(ble_spi, &char_00, 1, (uint8_t*)&read_char, 1);
      HAL_SPI_TransmitReceive(&hspi1, &char_00, (uint8_t*)&read_char, 1, BUS_SPI1_POLL_TIMEOUT);
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
//  HCI_TL_SPI_Enable_IRQ();

  /* Release CS line */
  rt_pin_write(HCI_TL_SPI_CS_PIN, PIN_HIGH);

//  rt_kprintf("Recv size: %d buffer: %02x", size, buffer[0]);
//  for (int i = 1; i < size; ++i) {
//      rt_kprintf("%02x",buffer[i]);
//  }
//  rt_kprintf("\r\n");

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

  rt_kprintf("Send size: %d buffer: %02x", size, buffer[0]);
  for (int i = 1; i < size; ++i) {
      rt_kprintf("%02x",buffer[i]);
  }
  rt_kprintf("\r\n");

  static uint8_t read_char_buf[MAX_BUFFER_SIZE];
  uint32_t tickstart = rt_tick_get();

//  HCI_TL_SPI_Disable_IRQ();

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
        rt_kprintf("!IsDataAvailable %d \r\n", IsDataAvailable());
      if((rt_tick_get() - tickstart_data_available) > TIMEOUT_DURATION)
      {
          rt_kprintf("%d timeout\r\n", rt_tick_get() - tickstart_data_available);
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
//    BSP_SPI1_SendRecv(header_master, header_slave, HEADER_SIZE);
//    rt_spi_send_then_recv(ble_spi, &header_master, HEADER_SIZE, &header_slave, HEADER_SIZE); //RTAPI
    HAL_SPI_TransmitReceive(&hspi1, header_master, header_slave, HEADER_SIZE, BUS_SPI1_POLL_TIMEOUT); //hal

    rt_kprintf("Send header size: %d buffer: %02x", HEADER_SIZE, header_slave[0]);
    for (int i = 1; i < HEADER_SIZE; ++i) {
        rt_kprintf("%02x", header_slave[i]);
    }
    rt_kprintf("\r\n");

    rx_bytes = (((uint16_t)header_slave[2])<<8) | ((uint16_t)header_slave[1]);
//    rx_bytes = (((uint16_t) 0x01 )<<8) | ((uint16_t) 0x09);

    if(rx_bytes >= size)
    {
      /* Buffer is big enough */
//      BSP_SPI1_SendRecv(buffer, read_char_buf, size);
//      rt_spi_send_then_recv(ble_spi, &buffer, size, &read_char_buf, size); //RTAPI
      HAL_SPI_TransmitReceive(&hspi1, buffer, read_char_buf, size, BUS_SPI1_POLL_TIMEOUT); //hal

//      rt_kprintf("Recv sd size: %d buffer: %02x", size, read_char_buf[0]);
//      for (int i = 1; i < size; ++i) {
//          rt_kprintf("%02x", read_char_buf[i]);
//      }
//      rt_kprintf("\r\n");

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
//  HCI_TL_SPI_Enable_IRQ();

//  rt_kprintf("SendRecv Rcv: %02x%02x%02x%02x%02x%02x%02x%02x%02x\r\n", read_char_buf[0], read_char_buf[1], read_char_buf[2],
//          read_char_buf[3], read_char_buf[4], read_char_buf[5], read_char_buf[6], read_char_buf[7], read_char_buf[8]);
//  rt_kprintf("\r\n");

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
  return (rt_pin_read(HCI_TL_SPI_EXTI_PIN) == PIN_HIGH);
}

/**
  * @brief  Send and Receive data to/from SPI BUS (Full duplex)
  * @param  pData: Pointer to data buffer to send/receive
  * @param  Length: Length of data in byte
  * @retval BSP status
  */
int32_t BSP_SPI1_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t Length)
{
  int32_t ret = RT_EOK;

  if (RT_EOK != rt_spi_send_then_recv(ble_spi, pTxData, Length, pRxData, Length)){
      rt_kprintf("Failed to send.");
      ret = - RT_ERROR;
  }
  return ret;
}

void send_cmd(uint16_t ogf, uint16_t ocf, uint8_t plen, void *param)
{
  uint8_t payload[HCI_MAX_PAYLOAD_SIZE];
  struct hci_command_hdr hc;

  hc.opcode = htobs(cmd_opcode_pack(ogf, ocf));
  hc.plen = plen;

  payload[0] = HCI_COMMAND_PKT;
  memcpy(payload + 1, &hc, sizeof(hc));
  memcpy(payload + HCI_HDR_SIZE + HCI_COMMAND_HDR_SIZE, param, plen);
  //payload: HCI_COMMAND_PKT(8)  opcode(16) plen(8) param(plen)
  //HCI_TL_SPI_Send(payload, HCI_HDR_SIZE + HCI_COMMAND_HDR_SIZE + plen)

  HCI_TL_SPI_Send(payload, HCI_HDR_SIZE + HCI_COMMAND_HDR_SIZE + plen);
}


/**
  * @brief  Initializes SPI HAL.
  * @retval BSP status
  */
static void SPI1_MspInit(SPI_HandleTypeDef* spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Enable Peripheral clock */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    PB3 (JTDO-TRACESWO)     ------> SPI1_SCK
    */
    GPIO_InitStruct.Pin = BUS_SPI1_MISO_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BUS_SPI1_MISO_GPIO_AF;
    HAL_GPIO_Init(BUS_SPI1_MISO_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUS_SPI1_MOSI_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BUS_SPI1_MOSI_GPIO_AF;
    HAL_GPIO_Init(BUS_SPI1_MOSI_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUS_SPI1_SCK_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BUS_SPI1_SCK_GPIO_AF;
    HAL_GPIO_Init(BUS_SPI1_SCK_GPIO_PORT, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
}

__weak HAL_StatusTypeDef MX_SPI1_Init(SPI_HandleTypeDef* hspi)
{
  HAL_StatusTypeDef ret = HAL_OK;

  hspi->Instance = SPI1;
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
  hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial = 7;
  hspi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(hspi) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  return ret;
}

int32_t BSP_SPI1_Init(void)
{
  int32_t ret = 0;

  hspi1.Instance  = SPI1;

  if(SPI1InitCounter++ == 0)
  {
    if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_RESET)
    {
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 0U)
        /* Init the SPI Msp */
        SPI1_MspInit(&hspi1);
#else
        if(IsSPI1MspCbValid == 0U)
        {
            if(BSP_SPI1_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
            {
                return BSP_ERROR_MSP_FAILURE;
            }
        }
#endif
        if(ret == 0)
        {
            /* Init the SPI */
            if (MX_SPI1_Init(&hspi1) != HAL_OK)
            {
                ret = -1;
            }
        }
    }
  }

  return ret;
}

int32_t HCI_HAL_SPI_Init(void* pConf)
{
//  GPIO_InitTypeDef GPIO_InitStruct;
//
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//
//  /* Configure EXTI Line */
//  GPIO_InitStruct.Pin = HCI_HAL_SPI_EXTI_PIN;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(HCI_HAL_SPI_EXTI_PORT, &GPIO_InitStruct);
//
//  /* Configure RESET Line */
//  GPIO_InitStruct.Pin =  HCI_HAL_RST_PIN ;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(HCI_HAL_RST_PORT, &GPIO_InitStruct);
//
//  /* Configure CS */
//  GPIO_InitStruct.Pin = HCI_HAL_SPI_CS_PIN ;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(HCI_HAL_SPI_CS_PORT, &GPIO_InitStruct);
//  /* Deselect CS PIN for BlueNRG at startup to avoid spurious commands */
//  HAL_GPIO_WritePin(HCI_HAL_SPI_CS_PORT, HCI_HAL_SPI_CS_PIN, GPIO_PIN_SET);

  return BSP_SPI1_Init();
  //return 0;
}


#endif /* APPLICATIONS_HCI_INTERFACE_C_ */
