#ifndef __SPO2_SENSOR_H__
#define __SPO2_SENSOR_H__


#include "Sensor.h"

/*----------------------------------------------------------------
 * SpO2 sensor
 * Singleton object, should only be created once
 * Interface defined in Sensor.h
 *---------------------------------------------------------------*/


/*!
 * @brief Create spO2 sensor
 *
 * @return Pointer to spO2 sensor
 */
__volatile__ Sensor * new_spo2_sensor(void);

#endif
