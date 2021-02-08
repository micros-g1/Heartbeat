#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "defs.h"

/*----------------------------------------------------------------
 * Generic sensor interface
 * Each sensor should define the init, start_sampling and 
 * stop_sampling functions
 *---------------------------------------------------------------*/

typedef struct {
	sensor_t type;
	void (* init)(uint32_t);
	void (* start_sampling)();
	void (* stop_sampling)();
	bool status;
} Sensor;


/*!
 * @brief Initializes sensor functionality
 */
void sensor_init();

/*!
 * @brief Send data to sensor queue
 *
 * @param sample : value of event
 * @param event_type : type of event
 * @param pxHigherPriorityTaskWoken : if not NULL, will use 
 * interrupt-safe ISR and return value to pass to portYIELD_FROM_ISR
 * 
 * @return True if write operation successful
 */
bool write_sample(float sample, sensor_event_type_t event_type, BaseType_t * pxHigherPriorityTaskWoken);

/*!
 * @brief Read data from sensor queue
 *
 * @param event : returns read event if operation successful
 * 
 * @return True if read operation successful
 */
bool read_sample(sensor_event_t * event);

/*!
 * @brief Set range for event type
 *
 * @param ev : event type
 * @param min : lower threshold of range
 * @param max : higher threshold of range
 */
void set_limits(sensor_event_type_t ev, float min, float max);

/*!
 * @brief Get current range status for event type
 *
 * @param ev : value of event
 * 
 * @return Range status (underflow, overflow, error, ok)
 */
uint32_t get_range_status(sensor_event_type_t ev);

/*!
 * @brief Update range status for event type from sample
 *
 * @param ev : event to update range
 * 
 * @return New range status (underflow, overflow, error, ok)
 */
uint32_t in_range(sensor_event_t ev);



#endif
