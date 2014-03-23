#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/hokuyo_tools.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/robot_parameters.h"
#include "kernel/math/regression.h"
#include "kernel/math/segment_intersection.h"
#include "kernel/math/polyline.h"
#include "kernel/location/location.h"
#include "gpio.h"
#include "table.h"
#include "detection.h"

//! @todo réglage au pif
#define DETECTION_STACK_SIZE         400
#define DETECTION_NUM_OBJECT         100
#define DETECTION_QUEUE_SIZE          20
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
static void detection_compute();
static void detection_remove_static_elements_from_dynamic_list();
static float detection_get_segment_similarity(const vect2* a, const vect2* b, const vect2* m, const vect2* n);
static void detection_hokuyo_callback();
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
static struct polyline detection_object[DETECTION_NUM_OBJECT];
static int32_t detection_num_obj;
static int detection_sick_state;
static struct fx_vect2 detection_sick_seg[2];

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

	hokuyo[HOKUYO1].register_callback(detection_hokuyo_callback);

	// TODO enregistrer sick

	detection_reg_size = 0;
/*
	hokuyo_scan.sens = PARAM_FOO_HOKUYO_SENS;
	hokuyo_scan.pos_hokuyo.x = PARAM_FOO_HOKUYO_X;
	hokuyo_scan.pos_hokuyo.y = PARAM_FOO_HOKUYO_Y;
	hokuyo_scan.pos_hokuyo.alpha = PARAM_FOO_HOKUYO_ALPHA;
	hokuyo_scan.pos_hokuyo.ca = fx_cos(hokuyo_scan.pos_hokuyo.alpha);
	hokuyo_scan.pos_hokuyo.sa = fx_sin(hokuyo_scan.pos_hokuyo.alpha);
*/
	return 0;
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
			//xSemaphoreTake(hokuyo_scan_mutex, portMAX_DELAY);
			if( event == DETECTION_EVENT_HOKUYO_1 )
			{
		//		struct systime last_time = systick_get_time();
				detection_compute();
		//		struct systime current_time = systick_get_time();
		//		struct systime dt = timediff(current_time, last_time);
		//		log_format(LOG_INFO, "compute_time : %lu us", dt.ms * 1000 + dt.ns/1000);
			}
#if 0
			if( event == DETECTION_EVENT_SICK )
			{
				detection_sick_state = get_sick(SICK_RIGHT | SICK_LEFT);

				if(detection_sick_state)
				{
					struct fx_vect2 a = { -200 << 16,  PARAM_LEFT_CORNER_Y };
					struct fx_vect2 b = { -200 << 16, PARAM_RIGHT_CORNER_Y };

					switch(detection_sick_state)
					{
						case SICK_RIGHT:
							a.x = PARAM_NP_X;
							a.y = -250<<16;
							b.x = -200<<16;
							b.y = 0;
							break;
						case SICK_LEFT:
							a.x = PARAM_NP_X;
							a.y = 250<<16;
							b.x = -200<<16;
							b.y = 0;
							break;
						default:
							break;
					}

					vect2_loc_to_abs(&pos, &a, detection_sick_seg);
					vect2_loc_to_abs(&pos, &b, detection_sick_seg + 1);
				}

				log_format(LOG_INFO, "sick = %d", detection_sick_state);
			}

			// ajout d'un segment pour gérer les sick
			if( detection_sick_state )
			{
				// ajout du segment
				detection_object[detection_num_obj].size = 2;
				detection_object[detection_num_obj].pt = detection_sick_seg;
				detection_num_obj++;
			}
#endif
			//TODOxSemaphoreGive(hokuyo_scan_mutex);

			// on limite la fréquence de l'envoi au cas ou.
			//if( detection_reg_size && event == DETECTION_EVENT_HOKUYO)
			{
				int i = 0;
				int16_t detect_size = 0;
				for(; i < detection_num_obj; i++)
				{
					if (detection_object[i].size)
					{
						detect_size++;
					}
				}

				usb_add(USB_DETECTION_DYNAMIC_OBJECT_SIZE, &detect_size, sizeof(detect_size));

				i = 0;
				for( ; i < detection_num_obj; i++)
				{
					usb_add(USB_DETECTION_DYNAMIC_OBJECT, detection_object[i].pt, sizeof(detection_object[i].pt[0]) * detection_object[i].size);
				}
			}

			detection_callback_function();
		}
	}
}

void detection_register_callback(detection_callback callback)
{
	detection_callback_function = callback;
}

static void detection_hokuyo_callback()
{
	unsigned char event = DETECTION_EVENT_HOKUYO_1;
	xQueueSend(detection_queue, &event, 0);
}

portBASE_TYPE isr_sick_update()
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	unsigned char event = DETECTION_EVENT_SICK;
	xQueueSendFromISR(detection_queue, &event, &xHigherPriorityTaskWoken);
	return xHigherPriorityTaskWoken;
}
#if 0
static int32_t detection_compute_object_on_trajectory(VectPlan* pos, const struct polyline* polyline, int size, vect2* a, vect2* b, float dist_min)
{
	vect2 a1 = { dist_min, PARAM_RIGHT_CORNER_Y };
	vect2 b1 = { 1e30, PARAM_RIGHT_CORNER_Y };
	vect2 a2 = { dist_min, PARAM_LEFT_CORNER_Y };
	vect2 b2 = { 1e30, PARAM_LEFT_CORNER_Y };

	vect2 c;
	vect2 d;
	vect2 h;

	int i;
	int j;
	float x_min = 1e30;
	float y_c = 0;
	float y_d = 0;

	for(i = 0; i < size; i++)
	{
		vect2_abs_to_loc(pos, &polyline[i].pt[0], &c);
		for(j = 1; j < polyline[i].size; j++)
		{
			vect2_abs_to_loc(pos, &polyline[i].pt[j], &d);

			// point c devant le robot et dans le tube
			if( c.x > dist_min && c.y > a1.y && c.y < a2.y)
			{
				if( c.x < x_min)
				{
					x_min = c.x;
					y_c = c.y;
					y_d = d.y;
				}
			}

			// point d devant le robot et dans le tube
			if( d.x > dist_min && d.y > a1.y && d.y < a2.y)
			{
				if( d.x < x_min)
				{
					x_min = d.x;
					y_c = c.y;
					y_d = d.y;
				}
			}

			// gestion du cas c et/ou d ne sont pas dans le tube
			int err1 = segment_intersection(a1, b1, c, d, &h);
			if(err1 == 0)
			{
				if( h.x < x_min)
				{
					x_min = h.x;
					y_c = c.y;
					y_d = d.y;
				}
			}

			int err2 = segment_intersection(a2, b2, c, d, &h);
			if(err2 == 0)
			{
				if( h.x < x_min)
				{
					x_min = h.x;
					y_c = c.y;
					y_d = d.y;
				}
			}

			c = d;
		}
	}

	if(y_c != y_d)
	{
		b1.x = x_min;
		b2.x = x_min;

		if( y_c < y_d)
		{
			b1.y = y_c;
			b2.y = y_d;
		}
		else
		{
			b1.y = y_d;
			b2.y = y_c;
		}
	}

	vect2_loc_to_abs(pos, &b1, a);
	vect2_loc_to_abs(pos, &b2, b);

	return x_min;
}
#endif
float detection_compute_front_object(enum detection_type type, const VectPlan* pos, const vect2* a, const vect2* b, float dist_min)
{
	float x_min = 1e30;
#if 0
	float x_min_table = 1e30;
	struct fx_vect2 c;
	struct fx_vect2 d;

	if(type == DETECTION_FULL || type == DETECTION_DYNAMIC_OBJ)
	{
		xSemaphoreTake(detection_mutex, portMAX_DELAY);
		x_min = detection_compute_object_on_trajectory(pos, detection_object, detection_num_obj, a, b, dist_min);
		xSemaphoreGive(detection_mutex);
	}
	else
	{
		c.x = x_min;
		d.y = PARAM_LEFT_CORNER_Y;

		d.x = x_min;
		d.y = PARAM_RIGHT_CORNER_Y;

		vect2_loc_to_abs(pos, &c, a);
		vect2_loc_to_abs(pos, &d, b);
	}

	if(type == DETECTION_FULL || type == DETECTION_STATIC_OBJ)
	{
		x_min_table = detection_compute_object_on_trajectory(pos, table_obj, TABLE_OBJ_SIZE, &c, &d, dist_min);

		if(x_min_table < x_min)
		{
			x_min = x_min_table;
			*a = c;
			*b = d;
		}
	}
#endif
	return x_min;
}

static void detection_compute()
{
	int i;

	// scan et position des points en x,y privé à la tache hokuyo
	hokuyo_compute_xy(&hokuyo[HOKUYO1].scan, detection_hokuyo_pos);

	// section critique - objets et segments partagés par les méthodes de calcul et la tache de mise à jour
	xSemaphoreTake(detection_mutex, portMAX_DELAY);
	detection_num_obj = hokuyo_find_objects(hokuyo[HOKUYO1].scan.distance, detection_hokuyo_pos, HOKUYO_NUM_POINTS, detection_object, DETECTION_NUM_OBJECT);
	detection_reg_size = 0;

	for( i = 0 ; i < detection_num_obj ; i++)
	{
		detection_object[i].size = regression_poly(detection_object[i].pt, detection_object[i].size, detection_reg_ecart, detection_hokuyo_reg + detection_reg_size, HOKUYO_REG_SEG - detection_reg_size);
		detection_object[i].pt = &detection_hokuyo_reg[detection_reg_size];
		detection_reg_size += detection_object[i].size;
	}
	detection_remove_static_elements_from_dynamic_list();
	xSemaphoreGive(detection_mutex);
}

static void detection_remove_static_elements_from_dynamic_list()
{
	int32_t nb_objects_to_test = detection_num_obj;
	int32_t i,j,k,l;
	
	//pour chaque objet détecté
	for(i=0; i<nb_objects_to_test; i++)
	{
		struct polyline* current_dyn_object=&detection_object[i];
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
						current_dyn_object->pt +j-1, current_dyn_object->pt +j,
						table_obj[k].pt +l-1, table_obj[k].pt +l);

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
					detection_object[detection_num_obj].pt=(current_dyn_object->pt)+j;
					detection_object[detection_num_obj].size=(current_dyn_object->size)-j;
					current_dyn_object->size=j;
					current_dyn_object=&(detection_object[detection_num_obj]);
					detection_num_obj++;
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
			if(current_dyn_object == (detection_object+detection_num_obj-1))
			{
				detection_num_obj--;
			}
		}
	}
}

//méthode heuristique pour estimer une resemblance entre deux segments
static float detection_get_segment_similarity(const vect2* a, const vect2* b, const vect2* m, const vect2* n)
{
	float similarity = 0;
//	vect2 ab = b - a;
//	vect2 mn = n - m;
	similarity += distance_point_to_segment(*a, *m, *n);
	similarity += distance_point_to_segment(*b, *m, *n);
//	similarity += cross_product_z(ab, mn)/mn.norm2();

	return similarity;
}