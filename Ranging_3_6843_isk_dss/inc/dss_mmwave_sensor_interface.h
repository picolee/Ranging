/*
 * mmwave_sensor_interface_dss.h
 *
 *  Created on: May 26, 2024
 *      Author: LeeLemay
 */

#ifndef INC_DSS_MMWAVE_SENSOR_INTERFACE_H_
#define INC_DSS_MMWAVE_SENSOR_INTERFACE_H_

#include <ti/control/mmwave/mmwave.h>
#include <ti/sysbios/knl/Task.h>

int16_t startSensor();
int32_t initializeMMWaveSystem();
void Ranging_dssMMWaveCtrlTask(UArg arg0, UArg arg1);


#endif /* INC_DSS_MMWAVE_SENSOR_INTERFACE_H_ */
