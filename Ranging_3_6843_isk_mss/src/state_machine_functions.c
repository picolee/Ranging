/*
 * state_machine_functions.c
 *
 *  Created on: Nov 7, 2023
 *      Author: Lee Lemay
 */

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  INCLUDES
 * ----------------------------------------------------------------------------------------------------------------- */

#include <inc/state_machine_definitions.h>
#include <inc/state_machine_functions.h>
#include <stdio.h>

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  DEFINES
 * ----------------------------------------------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Global Variables
 * ----------------------------------------------------------------------------------------------------------------- */

extern const char *  message_to_string_table[STATE_MACHINE_MSG_TOTAL_COUNT];
extern const char *  state_to_string_table[STATE_TOTAL_COUNT];
extern State_Information_Ptr_t g_ptr_Current_State_Info;

// Define generic behavior that occurs whenever we receive an invalid event
void Service_Null_Message( uint16_t message_flag )
{
    System_printf( "NULL flag %#X %s received for state %u\n",
      message_flag,
      message_to_string_table[message_flag],
      g_ptr_Current_State_Info->stateNumber );
}

void Send_Message( uint16_t message_flag )
{
  if (g_ptr_Current_State_Info->stateNumber >= STATE_TOTAL_COUNT)
  {
      System_printf( "State %u higher than STATE_TOTAL_COUNT: %u\n",
        g_ptr_Current_State_Info->stateNumber,
        STATE_TOTAL_COUNT);
  }
  else if (g_ptr_Current_State_Info->stateNumber >= STATE_TOTAL_COUNT &&
      g_ptr_Current_State_Info->stateNumber != g_Completed_State_info.stateNumber &&
      g_ptr_Current_State_Info->stateNumber != g_Failed_State_info.stateNumber &&
      g_ptr_Current_State_Info->stateNumber != g_Cancelled_State_info.stateNumber)
  {
      System_printf( "State %u higher than STATE_TOTAL_COUNT: %u\n",
        g_ptr_Current_State_Info->stateNumber,
        STATE_TOTAL_COUNT);
  }
  else
  {
      System_printf( "State %u: %s sent flag %#X %s\n",
        g_ptr_Current_State_Info->stateNumber,
        state_to_string_table[g_ptr_Current_State_Info->stateNumber],
        message_flag,
        message_to_string_table[message_flag] );
  }

  MsgObj msg;
  msg.id = message_flag;

  if (Mailbox_post(mbxHandle, &msg, BIOS_NO_WAIT))
  {
      System_printf("Mailbox Write: ID = %d.\n", msg.id);
  }
  else
  {
      System_printf("Mailbox Write Failed: ID = %d.\n", msg.id);
  }
}

void Leaving_State( State_Information_Ptr_t ptr_phase_information )
{
    System_printf( "HAICU end state %u: %s\n",
      ptr_phase_information->stateNumber,
      state_to_string_table[ptr_phase_information->stateNumber] );
}

void Entering_State( State_Information_Ptr_t ptr_phase_information )
{
  // If we are in a global phase, make sure it knows our current task
  if( ptr_phase_information->stateNumber == STATE_NUMBER_COMPLETE ||
      ptr_phase_information->stateNumber == STATE_NUMBER_FAIL ||
      ptr_phase_information->stateNumber == STATE_NUMBER_CANCELLED )
  {
    ptr_phase_information->stateNumber = ptr_phase_information->previousStateInfo_ptr->stateNumber;
  }

  System_printf( "HAICU start state %u: %s\n",
      ptr_phase_information->stateNumber,
      state_to_string_table[ptr_phase_information->stateNumber] );
}

void State_Machine_Initialization( State_Information_Ptr_t ptr_phase_information )
{
    System_printf("OK");
}
void State_Machine_Standby( State_Information_Ptr_t ptr_phase_information )
{
    System_printf("OK");
}
void State_Machine_Configure_Transmit( State_Information_Ptr_t ptr_phase_information )
{
    System_printf("OK");
}
void State_Machine_Transmit_Start_Code( State_Information_Ptr_t ptr_phase_information )
{
    System_printf("OK");
}
void State_Machine_Transmit_Response_Code( State_Information_Ptr_t ptr_phase_information )
{
    System_printf("OK");
}
void State_Machine_Configure_Receive( State_Information_Ptr_t ptr_phase_information )
{
    System_printf("OK");
}
void State_Machine_Receive_Start_Code( State_Information_Ptr_t ptr_phase_information )
{
    System_printf("OK");
}
void State_Machine_Receive_Response_Code( State_Information_Ptr_t ptr_phase_information )
{
    System_printf("OK");
}

void State_Machine_Task_Completed_Successfully( State_Information_Ptr_t ptr_phase_information )
{
    System_printf("OK");
}

void State_Machine_Task_Failed( State_Information_Ptr_t ptr_phase_information )
{
    System_printf("**********FAILURE:");
}

void State_Machine_Task_Cancelled( State_Information_Ptr_t ptr_phase_information )
{
    System_printf("**********Cancelled");
}
