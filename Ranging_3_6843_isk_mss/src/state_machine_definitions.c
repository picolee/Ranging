/*
 * state_machine_definitions.c
 *
 *  Created on: Nov 7, 2023
 *      Author: Lee Lemay
 */


/*
 * state_machine_state_definitions.c
 *
 *  Created on: Oct 19, 2023
 *      Author: lee lemay
 */

#include <inc/state_machine_definitions.h>
#include <inc/state_machine_functions.h>

const char *  message_to_string_table[STATE_MACHINE_MSG_TOTAL_COUNT];
const char *  state_to_string_table[STATE_TOTAL_COUNT] = {0};

State_Information_Ptr_t g_ptr_Current_State_Info;
State_Information_t     g_Completed_State_info;
State_Information_t     g_Failed_State_info;
State_Information_t     g_Cancelled_State_info;


// This array holds the State_Information_t that define the behavior of each state
State_Information_t State_Machine_States[8]     = {0};

void Configure_Initial_State_To_Idle( void )
{
  // Start in the idle state
  g_ptr_Current_State_Info = &State_Machine_States[0];
}

void Define_Global_States( void )
{
  // Configure the global states
  g_Completed_State_info.stateExecutionFunction = State_Machine_Task_Completed_Successfully;
  g_Completed_State_info.stateTransitionTable[STATE_MACHINE_MSG_GO_STANDBY] = &State_Machine_States[0];
  g_Completed_State_info.stateNumber = STATE_NUMBER_COMPLETE;

  g_Failed_State_info.stateExecutionFunction = State_Machine_Task_Failed;
  g_Failed_State_info.stateTransitionTable[STATE_MACHINE_MSG_GO_STANDBY] = &State_Machine_States[0];
  g_Failed_State_info.stateNumber = STATE_NUMBER_FAIL;

  g_Cancelled_State_info.stateExecutionFunction = State_Machine_Task_Cancelled;
  g_Cancelled_State_info.stateTransitionTable[STATE_MACHINE_MSG_GO_STANDBY] = &State_Machine_States[0];
  g_Cancelled_State_info.stateNumber = STATE_NUMBER_CANCELLED;

  // Configure the string table
  message_to_string_table[STATE_MACHINE_MSG_GO_STANDBY] = "MSG_GO_STANDBY";
  message_to_string_table[STATE_MACHINE_MSG_CANCELLED]  = "MSG_CANCELLED";
  message_to_string_table[STATE_MACHINE_MSG_FAILED]     = "MSG_FAILED";
  message_to_string_table[STATE_MACHINE_MSG_FAILED]     = "MSG_COMPLETED";

  state_to_string_table[STATE_INIT]                     = "INIT";
  state_to_string_table[STATE_STANDBY]                  = "STANDBY";
  state_to_string_table[STATE_CONFIGURE_RECEIVE]        = "CFG_RX";
  state_to_string_table[STATE_CONFIGURE_TRANSMIT]       = "CFG_TX";
  state_to_string_table[STATE_TRANSMIT_START_CODE]      = "TX_START_CODE";
  state_to_string_table[STATE_RECEIVE_START_CODE]       = "RX_START_CODE";
  state_to_string_table[STATE_TRANSMIT_RESPONSE_CODE]   = "TX_RSP_CODE";
  state_to_string_table[STATE_RECEIVE_RESPONSE_CODE]    = "RX_RSP_CODE";
}

void Define_State_Machine( void )
{
  // Initialization
  State_Machine_States[0].stateExecutionFunction = State_Machine_Initialization;
  State_Machine_States[0].stateNumber = 0;
  State_Machine_States[0].previousStateInfo_ptr = &State_Machine_States[0];
  State_Machine_States[0].stateTransitionTable[STATE_MACHINE_MSG_GO_STANDBY]    = &State_Machine_States[0];
  State_Machine_States[0].stateTransitionTable[STATE_MACHINE_MSG_COMPLETED]     = &State_Machine_States[1];
  State_Machine_States[0].stateTransitionTable[STATE_MACHINE_MSG_FAILED]        = &g_Failed_State_info;
  State_Machine_States[0].stateTransitionTable[STATE_MACHINE_MSG_CANCELLED]     = &g_Cancelled_State_info;

  // Standby
  State_Machine_States[1].stateExecutionFunction = State_Machine_Standby;
  State_Machine_States[1].stateNumber = 1;
  State_Machine_States[1].stateTransitionTable[STATE_MACHINE_MSG_GO_STANDBY]    = &State_Machine_States[0];
  State_Machine_States[1].stateTransitionTable[STATE_MACHINE_MSG_COMPLETED]     = &State_Machine_States[2];
  State_Machine_States[1].stateTransitionTable[STATE_MACHINE_MSG_FAILED]        = &g_Failed_State_info;
  State_Machine_States[1].stateTransitionTable[STATE_MACHINE_MSG_CANCELLED]     = &g_Cancelled_State_info;

  // Configure TX
  State_Machine_States[2].stateExecutionFunction = State_Machine_Configure_Transmit;
  State_Machine_States[2].stateNumber = 2;
  State_Machine_States[2].stateTransitionTable[STATE_MACHINE_MSG_GO_STANDBY]    = &State_Machine_States[0];
  State_Machine_States[2].stateTransitionTable[STATE_MACHINE_MSG_COMPLETED]     = &State_Machine_States[2];
  State_Machine_States[2].stateTransitionTable[STATE_MACHINE_MSG_FAILED]        = &g_Failed_State_info;
  State_Machine_States[2].stateTransitionTable[STATE_MACHINE_MSG_CANCELLED]     = &g_Cancelled_State_info;

  // TX Start Code
  State_Machine_States[3].stateExecutionFunction = State_Machine_Transmit_Start_Code;
  State_Machine_States[3].stateNumber = 3;
  State_Machine_States[3].stateTransitionTable[STATE_MACHINE_MSG_GO_STANDBY]  = &State_Machine_States[0];
  State_Machine_States[3].stateTransitionTable[STATE_MACHINE_MSG_COMPLETED]   = &State_Machine_States[3];
  State_Machine_States[3].stateTransitionTable[STATE_MACHINE_MSG_FAILED]      = &g_Failed_State_info;
  State_Machine_States[3].stateTransitionTable[STATE_MACHINE_MSG_CANCELLED]   = &g_Cancelled_State_info;

  // TX Response Code
  State_Machine_States[4].stateExecutionFunction = State_Machine_Transmit_Response_Code;
  State_Machine_States[4].stateNumber = 4;
  State_Machine_States[4].stateTransitionTable[STATE_MACHINE_MSG_GO_STANDBY]  = &State_Machine_States[0];
  State_Machine_States[4].stateTransitionTable[STATE_MACHINE_MSG_COMPLETED]   = &State_Machine_States[4]; // Cal coils
  State_Machine_States[4].stateTransitionTable[STATE_MACHINE_MSG_FAILED]      = &g_Failed_State_info;
  State_Machine_States[4].stateTransitionTable[STATE_MACHINE_MSG_CANCELLED]   = &g_Cancelled_State_info;

  // Configure RX
  State_Machine_States[5].stateExecutionFunction = State_Machine_Configure_Receive;
  State_Machine_States[5].stateNumber = 5;
  State_Machine_States[5].stateTransitionTable[STATE_MACHINE_MSG_GO_STANDBY]    = &State_Machine_States[0];
  State_Machine_States[5].stateTransitionTable[STATE_MACHINE_MSG_COMPLETED]     = &g_Completed_State_info;
  State_Machine_States[5].stateTransitionTable[STATE_MACHINE_MSG_FAILED]        = &g_Failed_State_info;
  State_Machine_States[5].stateTransitionTable[STATE_MACHINE_MSG_CANCELLED]     = &g_Cancelled_State_info;

  // RX Start Code
  State_Machine_States[6].stateExecutionFunction = State_Machine_Receive_Start_Code;
  State_Machine_States[6].stateNumber = 6;
  State_Machine_States[6].stateTransitionTable[STATE_MACHINE_MSG_GO_STANDBY]    = &State_Machine_States[0];
  State_Machine_States[6].stateTransitionTable[STATE_MACHINE_MSG_COMPLETED]     = &g_Completed_State_info;
  State_Machine_States[6].stateTransitionTable[STATE_MACHINE_MSG_FAILED]        = &g_Failed_State_info;
  State_Machine_States[6].stateTransitionTable[STATE_MACHINE_MSG_CANCELLED]     = &g_Cancelled_State_info;

  // RX Response Code
  State_Machine_States[7].stateExecutionFunction = State_Machine_Receive_Response_Code;
  State_Machine_States[7].stateNumber = 7;
  State_Machine_States[7].stateTransitionTable[STATE_MACHINE_MSG_GO_STANDBY]    = &State_Machine_States[0];
  State_Machine_States[7].stateTransitionTable[STATE_MACHINE_MSG_COMPLETED]     = &g_Completed_State_info;
  State_Machine_States[7].stateTransitionTable[STATE_MACHINE_MSG_FAILED]        = &g_Failed_State_info;
  State_Machine_States[7].stateTransitionTable[STATE_MACHINE_MSG_CANCELLED]     = &g_Cancelled_State_info;
}
