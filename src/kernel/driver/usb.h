#ifndef USB_H
#define USB_H

//! @file usb.h
//! @brief USB
//! @author Atlantronic

#ifdef __cplusplus
extern "C" {
#endif

enum
{
	USB_LOG = 1,
	USB_ERR = 2,
	USB_HOKUYO = 3,
	USB_CONTROL = 5,
	USB_GO = 6,
	USB_DETECTION_DYNAMIC_OBJECT_SIZE1 = 7,
	USB_DETECTION_DYNAMIC_OBJECT_SIZE2 = 8,
	USB_DETECTION_DYNAMIC_OBJECT1 = 9,
	USB_DETECTION_DYNAMIC_OBJECT2 = 10,
	USB_PUBLISH_VERSION = 11,
	USB_DATA_MAX = 12,     //!< nombre d'id, laisser en dernier
};

enum usb_cmd
{
	USB_CMD_PTASK = 1,
	USB_CMD_TRAJECTORY = 2,
	USB_CMD_MOTION_PARAM = 3,
	USB_CMD_MOTION_PRINT_PARAM = 4,
	USB_CMD_MOTION_MAX_SPEED = 5,
	USB_CMD_MOTION_GOTO = 6,
	USB_CMD_MOTION_SET_SPEED = 7,
	USB_CMD_MOTION_SET_MAX_CURRENT = 8,
	USB_CMD_MOTION_SET_ACTUATOR_KINEMATICS = 9,
	USB_CMD_MOTION_ENABLE = 10,
	USB_CMD_MOTION_HOMING = 11,
	USB_CMD_LOCATION_SET_POSITION = 12,
	USB_CMD_PINCE = 13,
	USB_CMD_DYNAMIXEL = 14,
	USB_CMD_RECALAGE = 15,
	USB_CMD_GO = 16,
	USB_CMD_MATCH_TIME = 17,
	USB_CMD_COLOR = 18,
	USB_CMD_ARM = 19,
	USB_CMD_STRAT = 20,
	USB_CMD_CAN_SET_BAUDRATE = 21,
	USB_CMD_CAN_WRITE = 22,
	USB_CMD_GYRO_CALIB = 23,
	USB_CMD_GYRO_SET_POSITION = 24,
	USB_CMD_GYRO_SET_CALIBRATION_VALUES = 25,
	USB_CMD_REBOOT = 26,
	USB_CMD_PUMP = 27,
	USB_CMD_XBEE = 28,
	USB_CMD_POWER = 29,
	USB_CMD_HEARTBEAT = 30,
	USB_CMD_LED_READY_FOR_INIT = 31,
    USB_CMD_REQUEST_VERSION = 32,
	USB_CMD_NUM = 33      //!< nombre de commandes, laisser en dernier
};

struct usb_header
{
	uint16_t type;
	uint16_t size;
} __attribute__((packed));

//!< ajout de log
//!< le module usb doit être initialisé
void usb_add(uint16_t type, void* msg, uint16_t size);

void usb_add_log(unsigned char level, const char* func, uint16_t line, const char* msg);

//!< ajout d'une commande
//!< peut être appelée avant l'initialisation du module usb
void usb_add_cmd(enum usb_cmd id, void (*cmd)(void*));

static inline unsigned char usb_is_get_version_done()
{
	extern unsigned char usb_get_version_done;
	return usb_get_version_done;
}

#ifdef __cplusplus
}
#endif

#endif
