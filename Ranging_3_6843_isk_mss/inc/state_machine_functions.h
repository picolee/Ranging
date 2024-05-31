/*
 * state_machine_functions.h
 *
 *  Created on: Nov 7, 2023
 *      Author: Lee Lemay
 */

#ifndef APPLICATION_HAICU_STATE_MACHINE_FUNCTIONS_H_
#define APPLICATION_HAICU_STATE_MACHINE_FUNCTIONS_H_

#include <inc/state_machine_definitions.h>

void Log_To_Uart(State_Information_Ptr_t p_stateInfo, const char* format, ...);
void Send_State_Machine_Message( uint16_t event_flag );
void Service_Null_Message( uint16_t event_flag );
void Leaving_State( State_Information_Ptr_t p_stateInfo );
void Entering_State( State_Information_Ptr_t p_stateInfo );

void SM_Func_Initialization( State_Information_Ptr_t p_stateInfo );
void SM_Func_Update_List_Of_Timeslots( State_Information_Ptr_t p_stateInfo );
void SM_Func_Standby( State_Information_Ptr_t p_stateInfo );
void SM_Func_Stop( State_Information_Ptr_t p_stateInfo );

void SM_Func_Cfg( State_Information_Ptr_t p_stateInfo );
void SM_Func_Activate_Cfg( State_Information_Ptr_t p_stateInfo );
void SM_Func_Execute_Cfg( State_Information_Ptr_t p_stateInfo );
void SM_Func_Executing( State_Information_Ptr_t p_stateInfo );
void SM_Func_Process_Result( State_Information_Ptr_t p_stateInfo );

void SM_Func_Cfg_Tx( State_Information_Ptr_t p_stateInfo );
void SM_Func_Activate_Tx_Cfg( State_Information_Ptr_t p_stateInfo );
void SM_Func_Tx_Start_Code( State_Information_Ptr_t p_stateInfo );
void SM_Func_Tx_Response_Code( State_Information_Ptr_t p_stateInfo );

void SM_Func_Cfg_Rx( State_Information_Ptr_t p_stateInfo );
void SM_Func_Activate_Rx_Cfg( State_Information_Ptr_t p_stateInfo );
void SM_Func_Rx( State_Information_Ptr_t p_stateInfo );
void SM_Func_Rx_Response_Code( State_Information_Ptr_t p_stateInfo );
void SM_Func_Rx_Sync_Complete( State_Information_Ptr_t p_stateInfo );

void SM_Func_Task_Completed_Successfully( State_Information_Ptr_t p_stateInfo );
void SM_Func_Task_Failed( State_Information_Ptr_t p_stateInfo );
void SM_Func_Task_Cancelled( State_Information_Ptr_t p_stateInfo );

#endif /* APPLICATION_HAICU_STATE_MACHINE_FUNCTIONS_H_ */
