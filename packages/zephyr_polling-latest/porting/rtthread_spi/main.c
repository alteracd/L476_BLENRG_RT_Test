
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <rtthread.h>

#include "rtthread_driver_spi.h"

#include "chipset_interface.h"
#include "platform_interface.h"

#include "base/types.h"
#include "utils/spool.h"
#include <logging/bt_log_impl.h>
#include <drivers/hci_driver.h>
#include "host/hci_core.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

extern void bt_ready(int err);
extern void app_polling_work(void);

int bt_init_hci_driver(void)
{
    printk("bt_init_hci_driver \r\n");

    bt_BlueNRG_SPI_interface_t *p_interface = NULL;
    uint8_t spi_num;
    uint8_t device_num;

    p_interface = (bt_BlueNRG_SPI_interface_t *)bt_BlueNRG_get_SPI_interface();
    bt_BlueNRG_SPI_interface_t tmp = {0, 0, 0, 0, 0, 0, 0, 0};
    tmp.cs_pin_num = p_interface->cs_pin_num;
    tmp.irq_pin_num = p_interface->irq_pin_num;
    tmp.rate = p_interface->rate;
    tmp.data_width = p_interface->data_width;
    tmp.LSB_MSB = p_interface->LSB_MSB;
    tmp.Master_Slave = p_interface->Master_Slave;
    tmp.CPOL = p_interface->CPOL;
    tmp.CPHA = p_interface->CPHA;

    spi_num = 1;
    device_num = 0;

    if (hci_driver_init(&tmp, device_num, spi_num) != 0)
    {
        printk("Error, SPI open failed.");
        return -1;
    }

    return 0;
}


void zephyr_polling_main(void* parameter)
{
    int err = 0;

    bt_log_impl_register(bt_log_impl_local_instance());

    if (bt_init_hci_driver() < 0)
    {
        return;
    }
    bt_hci_chipset_driver_register(bt_hci_chipset_impl_local_instance());
    bt_storage_kv_register(bt_storage_kv_impl_local_instance());
    bt_timer_impl_local_init();

    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(bt_ready);
    if(err)
    {
        printk("bt_enable(), err: %d\n", err);
    }

#if defined(CONFIG_BT_MONITOR_SLEEP)
    bt_init_monitor_sleep();
#endif

    while (1)
    {
#if defined(CONFIG_BT_MONITOR_SLEEP)
        if (!bt_check_is_in_sleep())
        {
            bt_polling_work();

            if (bt_is_ready() && bt_check_allow_sleep())
            {
                bt_sleep_prepare_work();
            }
        }
#else
        bt_polling_work();
#endif

        app_polling_work();

        extern void hci_driver_init_loop(void);
        hci_driver_init_loop();

        // rt_thread_yield();
        rt_thread_delay(1);
    }
}

int zephyr_polling_init(void)
{
    static rt_thread_t tid = RT_NULL;

    tid = rt_thread_create("zephyr_polling_main",
                            zephyr_polling_main, RT_NULL,
                            4096,
                            5, 5);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }

    return 0;
}
// INIT_APP_EXPORT(zephyr_polling_init);
MSH_CMD_EXPORT(zephyr_polling_init, "zephyr_polling start");


static int zephyr(void) {

    static rt_thread_t tid = RT_NULL;

    rt_kprintf("zephyr_polling_init \r\n");

    tid = rt_thread_create("zephyr_polling_main",
                                zephyr_polling_main, RT_NULL,
                                4096,
                                5, 5);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }

    return 0;
}
MSH_CMD_EXPORT(zephyr, "zephyr_polling_init sample");

