
#ifndef _CHIPSET_INTERFACE_H_
#define _CHIPSET_INTERFACE_H_

#include "drivers/hci_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t vid;
    uint16_t pid;
} bt_usb_interface_t;

typedef struct
{
    int rate;
    int databits;
    int stopbits;
    int parity;
    bool flowcontrol;
} bt_uart_interface_t;

typedef struct
{
    int cs_pin_num;         /* RT-Thread drv_gpio number of SPI CS PIN */
    int irq_pin_num;        /* RT-Thread drv_gpio number of SPI IRQ PIN */
    uint32_t rate;
    int data_width;         /* Data width: 8bits, 16bits, 32bits */
    int LSB_MSB;            /* Data transmission order: LSB:0 MSB:1 */
    int Master_Slave;       /* Set the master-slave mode of SPI:  Master:0 Slave:1 */
    int CPOL;               /* Set clock polarity(CPOL):  0 1 */
    int CPHA;               /* Set clock phase(CPHA):  0 1 */
} bt_BlueNRG_SPI_interface_t;

const bt_usb_interface_t *bt_chipset_get_usb_interface(void);
const bt_uart_interface_t *bt_chipset_get_uart_interface(void);
const bt_BlueNRG_SPI_interface_t *bt_BlueNRG_get_SPI_interface(void);

const struct bt_hci_chipset_driver *bt_hci_chipset_impl_local_instance(void);

#ifdef __cplusplus
}
#endif

#endif //_CHIPSET_INTERFACE_H_
