#ifndef __EKG_SENSOR_H__
#define __EKG_SENSOR_H__


#include "Sensor.h"

/*----------------------------------------------------------------
 * EKG sensor
 * Singleton object, should only be created once
 * Interface defined in Sensor.h
 *---------------------------------------------------------------*/


/*!
 * @brief Create EKG sensor
 *
 * @return Pointer to EKG sensor
 */
__volatile__ Sensor * new_ekg_sensor(void);

#endif
