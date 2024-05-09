/*
 * state_machine_functions.h
 *
 *  Created on: Nov 7, 2023
 *      Author: Lee Lemay
 */

#ifndef APPLICATION_HAICU_STATE_MACHINE_FUNCTIONS_H_
#define APPLICATION_HAICU_STATE_MACHINE_FUNCTIONS_H_

#include <inc/state_machine_definitions.h>

void HAICU_Send_Message( uint16_t message_flag );

void Service_Null_Message( uint16_t event_flag );
void Leaving_State( State_Information_Ptr_t ptr_phase_information );
void Entering_State( State_Information_Ptr_t ptr_phase_information );

void State_Machine_Initialization( State_Information_Ptr_t ptr_phase_information );
void State_Machine_Standby( State_Information_Ptr_t ptr_phase_information );
void State_Machine_Configure_Transmit( State_Information_Ptr_t ptr_phase_information );
void State_Machine_Transmit_Start_Code( State_Information_Ptr_t ptr_phase_information );
void State_Machine_Transmit_Response_Code( State_Information_Ptr_t ptr_phase_information );
void State_Machine_Configure_Receive( State_Information_Ptr_t ptr_phase_information );
void State_Machine_Receive_Start_Code( State_Information_Ptr_t ptr_phase_information );
void State_Machine_Receive_Response_Code( State_Information_Ptr_t ptr_phase_information );

void State_Machine_Task_Completed_Successfully( State_Information_Ptr_t ptr_phase_information );
void State_Machine_Task_Failed( State_Information_Ptr_t ptr_phase_information );
void State_Machine_Task_Cancelled( State_Information_Ptr_t ptr_phase_information );

#endif /* APPLICATION_HAICU_STATE_MACHINE_FUNCTIONS_H_ */
