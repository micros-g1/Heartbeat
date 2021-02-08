#ifndef __TEMPERATURE_SENSOR_H__
#define __TEMPERATURE_SENSOR_H__

#include "Sensor.h"

/*----------------------------------------------------------------
 * Temperature sensor
 * Singleton object, should only be created once
 * Interface defined in Sensor.h
 *---------------------------------------------------------------*/


/*!
 * @brief Create temperature sensor
 *
 * @return Pointer to temperature sensor
 */
__volatile__ Sensor * new_temperature_sensor(void);

#endif
