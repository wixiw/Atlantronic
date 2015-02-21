#ifndef USB_DESCRIPTOR_H
#define USB_DESCRIPTOR_H

#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05
#define USB_LEN_DEV_QUALIFIER_DESC              0x0A
#define USB_DATA_SUBCLASS                       0x00

#define USB_DEVICE_DESCRIPTOR_SIZE              0x12
#define USB_CONFIG_DESCRIPTOR_SIZE                32
#define USB_STRING_LANG_ID_SIZE                    4

extern const uint8_t usb_device_descriptor[USB_DEVICE_DESCRIPTOR_SIZE];
extern const uint8_t usb_config_descriptor[USB_CONFIG_DESCRIPTOR_SIZE];
extern const uint8_t usb_string_langID[USB_STRING_LANG_ID_SIZE];
extern uint8_t usb_device_qualifier_desc[USB_LEN_DEV_QUALIFIER_DESC];

#define USB_TX_EP_ID			   				   1
#define USB_TX_EP_ADDR			   				0x81
#define USB_RX_EP_ID			   				   2
#define USB_RX_EP_ADDR			   		        0x02
#define USBD_MANUFACTURER_STRING      "A.R.D."
#define USBD_PRODUCT_STRING           "stm32_ard"
#define USBD_SERIALNUMBER_STRING      "201500000001"
#define USBD_CONFIGURATION_STRING     "A.R.D. config"
#define USBD_INTERFACE_STRING         "A.R.D. interface"

#endif
