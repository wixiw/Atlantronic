#include "os/os.h"
#include "os/module.h"
#include "os/rcc.h"
#include "core/gpio.h"
#include "components/log/log.h"
#include "hokuyo_tools.h"
#include "hokuyo.h"
#include "regression.h"
#include "segment_intersection.h"
#include "polyline.h"
#include "location.h"
#include "detection.h"
#include "com/msgs/OpponentListMsg.hpp"
#include "com/stack_com/ArdCom.hpp"
#include "table.h"
#include "os/error_hook.h"

using namespace arp_stm32;

//! @todo réglage au pif
#define DETECTION_STACK_SIZE        1024
#define DETECTION_QUEUE_SIZE           2
#define HOKUYO_REG_SEG               200
#define SIMILARITY_ACCEPTANCE        200

enum
{
	DETECTION_EVENT_HOKUYO_1,
	DETECTION_EVENT_HOKUYO_2,
	DETECTION_EVENT_SICK
};

static void detection_task(void* arg);
static int detection_module_init();
static void detection_compute(int id);
static void detection_remove_static_elements_from_dynamic_list(int id);
static float detection_get_segment_similarity(const vect2* a, const vect2* b, const vect2* m, const vect2* n);
static void detection_hokuyo1_callback();
static void detection_hokuyo2_callback();
static xQueueHandle detection_queue;
static detection_callback detection_callback_function = (detection_callback)nop_function;

// données privées à la tache detection, ne doit pas être disponible
// à l'extérieur car ce n'est pas connu pour un hokuyo distant (sur bar)
// Les méthodes de calculs doivent utiliser les objets et segments
static vect2 detection_hokuyo_pos[HOKUYO_NUM_POINTS];
static int detection_reg_ecart = 25;

// données partagées par la tache et des méthodes d'accés
static xSemaphoreHandle detection_mutex;
static vect2 detection_hokuyo_reg[HOKUYO_REG_SEG];
static int detection_reg_size;
static struct polyline detection_object_polyline[DETECTION_NUM_OBJECT];
static struct detection_object detection_obj1[DETECTION_NUM_OBJECT];
static struct detection_object detection_obj2[DETECTION_NUM_OBJECT];
static int32_t detection_num_obj[HOKUYO_MAX];

int detection_module_init()
{
	portBASE_TYPE err = xTaskCreate(detection_task, "detect", DETECTION_STACK_SIZE, NULL, PRIORITY_TASK_DETECTION, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_DETECTION;
	}

	detection_mutex = xSemaphoreCreateMutex();

	if( ! detection_mutex )
	{
		return ERR_INIT_DETECTION;
	}

	detection_queue = xQueueCreate(DETECTION_QUEUE_SIZE, 1);

	if( ! detection_queue )
	{
		return ERR_INIT_DETECTION;
	}

	hokuyo[HOKUYO_AVANT].register_callback(detection_hokuyo1_callback);
	hokuyo[HOKUYO_ARRIERE].register_callback(detection_hokuyo2_callback);

	detection_reg_size = 0;

	return MODULE_INIT_SUCCESS;
}

module_init(detection_module_init, INIT_DETECTION);

static void detection_task(void* arg)
{
	(void) arg;
	unsigned char event;

	while(1)
	{
		// attente d'un evenement hokuyo ou sick
		if( xQueueReceive(detection_queue, &event, portMAX_DELAY) )
		{
			if( ArdCom::getInstance().isConnected() )
			{
				//xSemaphoreTake(hokuyo_scan_mutex, portMAX_DELAY);
				if( event == DETECTION_EVENT_HOKUYO_1 )
				{
					detection_compute(HOKUYO_AVANT);

					//TODO
					//xSemaphoreGive(hokuyo_scan_mutex);

					// on envoi les donnees par usb pour le debug
					OpponentListMsg msg(detection_obj1, detection_num_obj[HOKUYO_AVANT]);
					ArdCom::getInstance().send(msg);
				}
				if( event == DETECTION_EVENT_HOKUYO_2 )
				{
					detection_compute(HOKUYO_ARRIERE);

					//TODO
					//xSemaphoreGive(hokuyo_scan_mutex);

					OpponentListMsg msg(detection_obj2, detection_num_obj[HOKUYO_ARRIERE]);
					ArdCom::getInstance().send(msg);
				}

				detection_callback_function();
			}
		}
	}
}

void detection_register_callback(detection_callback callback)
{
	detection_callback_function = callback;
}

static void detection_hokuyo1_callback()
{
	unsigned char event = DETECTION_EVENT_HOKUYO_1;
	xQueueSend(detection_queue, &event, 0);
}

static void detection_hokuyo2_callback()
{
	unsigned char event = DETECTION_EVENT_HOKUYO_2;
	xQueueSend(detection_queue, &event, 0);
}

static void detection_compute(int id)
{
	int i;

	// scan et position des points en x,y privé à la tache hokuyo
	hokuyo_compute_xy(&hokuyo[id].scan, detection_hokuyo_pos);

	// section critique - objets et segments partagés par les méthodes de calcul et la tache de mise à jour
	xSemaphoreTake(detection_mutex, portMAX_DELAY);
	detection_num_obj[id] = hokuyo_find_objects(&hokuyo[id].scan, detection_hokuyo_pos, HOKUYO_NUM_POINTS, detection_object_polyline, DETECTION_NUM_OBJECT);
	detection_reg_size = 0;

	for( i = 0 ; i < detection_num_obj[id] ; i++)
	{
		detection_object_polyline[i].size = regression_poly(detection_object_polyline[i].pt, detection_object_polyline[i].size, detection_reg_ecart, detection_hokuyo_reg + detection_reg_size, HOKUYO_REG_SEG - detection_reg_size);
		detection_object_polyline[i].pt = &detection_hokuyo_reg[detection_reg_size];
		detection_reg_size += detection_object_polyline[i].size;
	}
	detection_remove_static_elements_from_dynamic_list(id);

	for( i = 0; i < detection_num_obj[id] ; i++)
	{
		float xmin = detection_object_polyline[i].pt[0].x;
		float xmax = xmin;
		float ymin = detection_object_polyline[i].pt[0].y;
		float ymax = ymin;
		for(int j = 1; j < detection_object_polyline[i].size; j++ )
		{
			if( detection_object_polyline[i].pt[j].x < xmin )
			{
				xmin = detection_object_polyline[i].pt[j].x;
			}
			else if( detection_object_polyline[i].pt[j].x > xmax )
			{
				xmax = detection_object_polyline[i].pt[j].x;
			}

			if( detection_object_polyline[i].pt[j].y < ymin )
			{
				ymin = detection_object_polyline[i].pt[j].y;
			}
			else if( detection_object_polyline[i].pt[j].y > ymax )
			{
				ymax = detection_object_polyline[i].pt[j].y;
			}
		}

		if(id == HOKUYO_AVANT )
		{
			detection_obj1[i].x = (xmin + xmax) / 2;
			detection_obj1[i].y = (ymin + ymax) / 2;
			detection_obj1[i].size = xmax - xmin;
			if( ymax - ymin > detection_obj1[i].size)
			{
				detection_obj1[i].size = ymax - ymin;
			}
		}
		else //arriere
		{
			detection_obj2[i].x = (xmin + xmax) / 2;
			detection_obj2[i].y = (ymin + ymax) / 2;
			detection_obj2[i].size = xmax - xmin;
			if( ymax - ymin > detection_obj2[i].size)
			{
				detection_obj2[i].size = ymax - ymin;
			}
		}
	}

	xSemaphoreGive(detection_mutex);
}

static void detection_remove_static_elements_from_dynamic_list(int id)
{
	int32_t nb_objects_to_test = detection_num_obj[id];
	int32_t i,j,k,l;
	
	//pour chaque objet détecté
	for(i=0; i<nb_objects_to_test; i++)
	{
		struct polyline* current_dyn_object=&detection_object_polyline[i];
		int8_t dynamic_segment_in_object = 0;
		//tester chaque segment (itération sur second point du segment)
		for(j=1; j< current_dyn_object->size; j++)
		{
			int8_t similar_static_segment_found = 0;
			//comparer aux segments statiques
			for(k=0; (k< TABLE_OBJ_SIZE)&&(!similar_static_segment_found); k++)
			{
				for(l=1; (l<table_obj[k].size)&&(!similar_static_segment_found); l++)
				{
					int32_t similarity = detection_get_segment_similarity(
						current_dyn_object->pt +j-1,
						current_dyn_object->pt +j,
						table_obj[k].pt +l-1,
						table_obj[k].pt +l);

					if ( similarity < SIMILARITY_ACCEPTANCE)
					{
						similar_static_segment_found = 1;
					}
				}
			}

			if(similar_static_segment_found)
			{
				//le vecteur appartient à un objet statique
				if(!dynamic_segment_in_object)
				{
					//On ampute l'objet du point précédent
					(current_dyn_object->pt)++;
					(current_dyn_object->size)--;
					j--;
				}
				else
				{
					//On réduit l'objet aux segments dynamiques précédents et
					//on reporte les segments non évalués dans un nouvel objet
					detection_object_polyline[detection_num_obj[id]].pt=(current_dyn_object->pt)+j;
					detection_object_polyline[detection_num_obj[id]].size=(current_dyn_object->size)-j;
					current_dyn_object->size=j;
					current_dyn_object=&(detection_object_polyline[detection_num_obj[id]]);
					detection_num_obj[id]++;
					j = 0;
					dynamic_segment_in_object = 0;
				}
			}
			else
			{
				//le vecteur n'appartient pas à un objet statique
				dynamic_segment_in_object = 1;
			}

		}
		if(current_dyn_object->size == 1)
		{
			//si un objet ne contient plus qu'un point, on l'élimine
			current_dyn_object->size=0;
			//si l'objet est en fin de liste, on libère cette position de la liste
			if(current_dyn_object == (detection_object_polyline+detection_num_obj[id]-1))
			{
				detection_num_obj[id]--;
			}
		}
	}
}

//méthode heuristique pour estimer une resemblance entre deux segments
static float detection_get_segment_similarity(const vect2* a, const vect2* b, const vect2* m, const vect2* n)
{
	ardAssert(NULL != a);
	ardAssert(NULL != b);
	ardAssert(NULL != m);
	ardAssert(NULL != n);

	float similarity = 0;
//	vect2 ab = b - a;
//	vect2 mn = n - m;
	similarity += distance_point_to_segment(*a, *m, *n);
	similarity += distance_point_to_segment(*b, *m, *n);
//	similarity += cross_product_z(ab, mn)/mn.norm2();

	return similarity;
}
