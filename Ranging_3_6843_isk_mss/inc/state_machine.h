/*
 * state_machine.h
 *
 *  Created on: Nov 7, 2023
 *      Author: Lee Lemay
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                     Includes
 * ----------------------------------------------------------------------------------------------------------------- */

#include <stdint.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/uart/UART.h>

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                     Defines
 * ----------------------------------------------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Global Exports
 * ----------------------------------------------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                     Typedefs
 * ----------------------------------------------------------------------------------------------------------------- */

void State_Machine_Init(uint8_t taskPriority, UART_Handle uart);

void Configure_State_Machine_Receive( float frequencyInGhz, uint16_t prn );
void Activate_State_Machine_Receive_Cfg( );
void Begin_State_Machine_Receive_Start_Code( );

void Configure_State_Machine_Transmit_Start_Code( float frequencyInGhz, uint16_t prn );
void Activate_State_Machine_Transmit_Cfg( );
void Begin_State_Machine_Transmit_Start_Code();
void Begin_State_Machine_Transmit_Response_Code();

void Send_State_Machine_Standby_Message( );
void Send_State_Machine_Init_Message( );

void Send_Code_Detect_Message();
void Send_No_Code_Detect_Message( );

void Send_Sensor_Started_Message( );

void Send_Transmit_Complete_Message();

void Send_DSS_Reports_Success_Message( );
void Send_DSS_Reports_Failure_Message( );

#endif /* APPLICATION_HAICU_STATE_MACHINE_H_ */
