//#include <errno.h>
//
//#include "chipset_common.h"
//
//// public API
//const struct bt_hci_chipset_driver *bt_hci_chipset_impl_local_instance(void)
//{
//    return NULL;
//}
//
//// For test, you can set your customor setting here.
//static const bt_usb_interface_t usb_interface = {0, 0};
//
//const bt_usb_interface_t *bt_chipset_get_usb_interface(void)
//{
//    return &usb_interface;
//}
//
//// For test, you can set your customor setting here.
//static const bt_uart_interface_t uart_interface = {1000000, 8, 1, 0, true};
//const bt_uart_interface_t *bt_chipset_get_uart_interface(void)
//{
//    return &uart_interface;
//}
