/*
 * ranging_dpc_interface.h
 *
 *  Created on: May 13, 2024
 *      Author: LeeLemay
 */

#ifndef INC_RANGING_DPC_INTERFACE_H_
#define INC_RANGING_DPC_INTERFACE_H_

int32_t Ranging_dataPathConfig (uint16_t rxPrn);
int32_t Ranging_DPM_ioctl_blocking
(
    DPM_Handle handle,
    uint32_t cmd,
    void* arg,
    uint32_t argLen
);



#endif /* INC_RANGING_DPC_INTERFACE_H_ */
