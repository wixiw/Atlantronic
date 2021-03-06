#ifndef HOKUYO_H
#define HOKUYO_H

//! @file hokuyo.h
//! @brief Hokuyo module
//! @author Atlantronic


#include <stdint.h>
#include "vect_plan.h"

#ifndef LINUX
#include "core/usart.h"
#include "os/module.h"
#include "os/os.h"
#else
#include "os/systime.h"
#endif

#define HOKUYO_NUM_POINTS            682
//!< taille de la réponse maxi avec hokuyo_scan_all :
//!< 682 points => 1364 data
//!< 1364 data = 21 * 64 + 20 data
//!< donc 23 octets entête, + 21*(64+2) + (20+2) + 1 = 1432
#define HOKUYO_SCAN_BUFFER_SIZE       1432

#define HOKUYO_DTHETA         	                                        (M_PI / 512.0f)
#define HOKUYO_START_ANGLE               ((- 135 * M_PI / 180.0f) + 44 * HOKUYO_DTHETA)      //!< 135 degrés + 44 HOKUYO_DTHETA
#define HOKUYO_MAX_RANGE                                                           4000
#define HOKUYO_POINT_TO_POINT_DT                                         (0.1f/1024.0f)

#ifdef __cplusplus
extern "C" {
#endif

enum hokuyo_id
{
	HOKUYO_AVANT = 0,
	HOKUYO_ARRIERE,
	HOKUYO_MAX,
};

struct hokuyo_scan
{
	int id;
	VectPlan pos_robot; //!< position absolue du robot au moment du scan
	VectPlan pos_hokuyo; //!< position du hokuyo dans le repère robot
	signed char sens; //!< sens du hokuyo (1 = vers le haut, -1 = vers le bas)
	uint16_t distance[HOKUYO_NUM_POINTS]; //!< distances des angles 44 à 725 du hokuyo
	systime date;
	float theta_min;
	float theta_max;
	int16_t min_object_size;
	int16_t min_distance;
} __attribute__((packed));

#ifndef LINUX

typedef void (*hokuyo_callback)();

class Hokuyo
{
	public:
		__OPTIMIZE_SIZE__ int init(enum usart_id id, const char* name, int hokuyo_id);
		void setPosition(VectPlan pos, int sens);

		//!< enregistrement d'une callback
		void register_callback(hokuyo_callback callback);

		xSemaphoreHandle scan_mutex;
		struct hokuyo_scan scan;

		//when set to true the hokuyo will send its raw scan on usb
		void setDebug(bool debug);

	protected:
		static void task_wrapper(void* arg);
		void task();
		uint32_t wait_decode_scan();
		__OPTIMIZE_SIZE__ uint32_t init_com();
		uint32_t scip2();
		uint32_t transaction(unsigned char* buf, uint32_t write_size, uint32_t read_size, portTickType timeout);
		uint32_t check_cmd(unsigned char* cmd, uint32_t size);
		uint32_t check_sum(uint32_t start, uint32_t end);
		uint32_t set_speed();
		uint32_t hs();
		uint32_t laser_on();
		int decode_scan();
		void fault_update(uint32_t err);
		void start_scan();

		static uint16_t decode16(const unsigned char* data);

		hokuyo_callback callback;
		uint32_t last_error;
		enum usart_id usartId;
		// variable alignee pour le dma
		uint8_t read_dma_buffer[HOKUYO_SCAN_BUFFER_SIZE] __attribute__ ((aligned (16)));

		bool m_debug;
};

extern struct Hokuyo hokuyo[HOKUYO_MAX];

#endif

#ifdef __cplusplus
}
#endif

#endif
