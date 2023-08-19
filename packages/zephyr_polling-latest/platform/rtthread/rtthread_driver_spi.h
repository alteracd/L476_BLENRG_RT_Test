/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-08-09     YTR       the first version
 */

#ifndef APPLICATIONS_HCI_INTERFACE_H_
#define APPLICATIONS_HCI_INTERFACE_H_

/* Defines -------------------------------------------------------------------*/
#define HEADER_SIZE       5U
#define MAX_BUFFER_SIZE   255U
#define TIMEOUT_DURATION  100U
#define TIMEOUT_IRQ_HIGH  1000U

#define HCI_PCK_TYPE_OFFSET             0
#define EVENT_PARAMETER_TOT_LEN_OFFSET  2


#define HCI_TL_SPI_EXTI_IRQn  EXTI0_IRQn

/*---------- Number of Bytes reserved for HCI Read Packet -----------*/
#define HCI_READ_PACKET_SIZE      128
/*---------- Number of Bytes reserved for HCI Max Payload -----------*/
#define HCI_MAX_PAYLOAD_SIZE      128

/* HCI Packet types */
#define HCI_COMMAND_PKT     0x01
#define HCI_ACLDATA_PKT     0x02
#define HCI_SCODATA_PKT     0x03
#define HCI_EVENT_PKT       0x04
#define HCI_VENDOR_PKT      0xff

/* Command opcode pack/unpack */
#define cmd_opcode_pack(ogf, ocf)   (uint16_t)((ocf & 0x03ff)|(ogf << 10))
#define cmd_opcode_ogf(op)      (op >> 10)
#define cmd_opcode_ocf(op)      (op & 0x03ff)

/* Byte order conversions */
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define htobs(d)  (d)
#define htobl(d)  (d)
#define htob(d,n) (d)
#define btohs(d)  (d)
#define btohl(d)  (d)
#define btoh(d,n) (d)
#elif __BYTE_ORDER == __BIG_ENDIAN
#define htobs(d)  (d<<8|d>>8)
#define htobl(d)  (d<<24|((d<<8)&0x00ff0000)|((d>>8)&0x0000ff00)|((d>>24)&0x000000ff))
#define htob(d,n) (d<<24|((d<<8)&0x00ff0000)|((d>>8)&0x0000ff00)|((d>>24)&0x000000ff))
#define btohs(d)  (d<<8|d>>8)
#define btohl(d)  (d<<24|((d<<8)&0x00ff0000)|((d>>8)&0x0000ff00)|((d>>24)&0x000000ff))
#define btoh(d,n) (d<<24|((d<<8)&0x00ff0000)|((d>>8)&0x0000ff00)|((d>>24)&0x000000ff))
#else
#error "Unknown byte order"
#endif

/* hci_command_hdr */
struct hci_command_hdr{
  uint16_t opcode;      /* OCF & OGF */
  uint8_t  plen;
};
#define HCI_COMMAND_HDR_SIZE    3
#define HCI_HDR_SIZE 1

/* GET PIN  -----------------------------------------------*/
//#define HCI_TL_SPI_EXTI_PIN     GET_PIN(A, 0)
#define HCI_TL_SPI_IRQ_PIN      GET_PIN(A, 0)
#define HCI_TL_SPI_CS_PIN       GET_PIN(A, 1)
//#define HCI_TL_RST_PIN          GET_PIN(A, 8)

/* HAL SPI && PIN -----------------------------------------------*/
#define HCI_HAL_SPI_EXTI_PORT  GPIOA
#define HCI_HAL_SPI_EXTI_PIN   GPIO_PIN_0
#define HCI_HAL_SPI_EXTI_IRQn  EXTI0_IRQn

#define HCI_HAL_SPI_IRQ_PORT   GPIOA
#define HCI_HAL_SPI_IRQ_PIN    GPIO_PIN_0

#define HCI_HAL_SPI_CS_PORT    GPIOA
#define HCI_HAL_SPI_CS_PIN     GPIO_PIN_1

#define HCI_HAL_RST_PORT       GPIOA
#define HCI_HAL_RST_PIN        GPIO_PIN_8

#define BUS_SPI1_INSTANCE SPI1
#define BUS_SPI1_MISO_GPIO_PIN GPIO_PIN_6
#define BUS_SPI1_MISO_GPIO_PORT GPIOA
#define BUS_SPI1_MISO_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()
#define BUS_SPI1_MISO_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_SPI1_MISO_GPIO_AF GPIO_AF5_SPI1
#define BUS_SPI1_MOSI_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_SPI1_MOSI_GPIO_PORT GPIOA
#define BUS_SPI1_MOSI_GPIO_PIN GPIO_PIN_7
#define BUS_SPI1_MOSI_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()
#define BUS_SPI1_MOSI_GPIO_AF GPIO_AF5_SPI1
#define BUS_SPI1_SCK_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_SPI1_SCK_GPIO_PORT GPIOB
#define BUS_SPI1_SCK_GPIO_PIN GPIO_PIN_3
#define BUS_SPI1_SCK_GPIO_AF GPIO_AF5_SPI1
#define BUS_SPI1_SCK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

#ifndef BUS_SPI1_POLL_TIMEOUT
  #define BUS_SPI1_POLL_TIMEOUT                   0x1000U
#endif


/* SPI  -----------------------------------------------*/
#define SPI_DEVICE_NAME     "spi10"   /* SPI Éè±¸Ãû³Æ */


/* Exported Functions --------------------------------------------------------*/
int32_t HCI_TL_SPI_Init    (void* pConf);
//int32_t HCI_TL_SPI_DeInit  (void);
int32_t HCI_TL_SPI_Receive (uint8_t* buffer, uint16_t size);
int32_t HCI_TL_SPI_Send    (uint8_t* buffer, uint16_t size);
int32_t HCI_TL_SPI_Reset   (void);
void send_cmd(uint16_t ogf, uint16_t ocf, uint8_t plen, void *param);
int32_t HCI_HAL_SPI_Init(void* pConf);

void hci_driver_h4_init(void);


//int32_t BSP_SPI1_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t Length);

#endif /* PACKAGES_ZEPHYR_POLLING_LATEST_PLATFORM_RTTHREAD_RTTHREAD_DRIVER_SPI_H_ */
