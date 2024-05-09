/*
 * state_machine.c
 * *
 * the dynamics of the State Machine are defined by:
 * * The current State_Information_t (defined in state_machine_definitions.h)
 * * The TI SYS/BIOS Mailbox messages that are received (defined in state_machine_definitions.h)
 *
 * The state machine is composed of an array of State_Information_t, which are the states for that task.
 * * Each state has state information like PRN, etc.
 * * It also contains a pointer to the function that executes that phase.
 * * Finally, it includes a state transition table, which maps received messages to the next state.
 *
 * Functions that implement the functionality are meant to be modular, reusable chunks of functionality, with the
 * state specific behavior encapsulated in the State_Information_t argument that is passed in
 *
 * In this framework, we compose a new state machine by defining a new mapping between
 * events and phase transitions. This will enable a lot of code re-use and provide
 * a straightforward mechanism to create new behaviors.
 *
 * ******************************************************
 * States
 * Each state in the state machine has its own function.
 * These phases can be composed to form complex behaviors.
 * New states can be added if new behaviors are needed.
 *
 * The states are:
 * STATE_INIT              = 0U,
 * STATE_STANDBY,
 * STATE_CONFIGURE_RECEIVE,
 * STATE_CONFIGURE_TRANSMIT,
 * STATE_TRANSMIT_START_CODE,
 * STATE_RECEIVE_START_CODE,
 * STATE_TRANSMIT_RESPONSE_CODE,
 * STATE_RECEIVE_RESPONSE_CODE,
 *
 * *******************************************************
 * Events
 *
 * Events are TI SYS/BIOS Mailbox messages that are posted to the state machine thread.
 * They are defined in state_machine.h
 *
 *  Created on: Nov 7, 2023
 *      Author: Lee Lemay
 */

#include <inc/state_machine.h>
#include <inc/state_machine_definitions.h>
#include <inc/state_machine_functions.h>

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Global Variables
 * ----------------------------------------------------------------------------------------------------------------- */

extern State_Information_Ptr_t g_ptr_Current_State_Info;

extern const char *  message_to_string_table[STATE_MACHINE_MSG_TOTAL_COUNT];
extern const char *  state_to_string_table[STATE_TOTAL_COUNT];


/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                   Local Variable Declarations
 * ----------------------------------------------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                   Local Function Declarations
 * ----------------------------------------------------------------------------------------------------------------- */

void Define_HAICU_State_Machines(void);
void State_Machine_Thread_Start(UArg arg0, UArg arg1);

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                   External Functions
 * ----------------------------------------------------------------------------------------------------------------- */

void State_Machine_Init(uint8_t taskPriority, Task_Handle* stateMachineTask)
{
    /* create threads */
    Task_Params         taskParams;
    Task_Params_init(&taskParams);
    taskParams.priority  = taskPriority;
    taskParams.stackSize = 4*1024;
    *stateMachineTask = Task_create(State_Machine_Thread_Start, &taskParams, NULL);

    // Create a Mailbox instance */
    Mailbox_Params mbxParams;
    Mailbox_Params_init(&mbxParams);
    mbxParams.buf     = (Ptr)mailboxBuffer;
    mbxParams.bufSize = sizeof(mailboxBuffer);
    Mailbox_construct(&mbxStruct, sizeof(MsgObj), NUMMSGS, &mbxParams, NULL);
    mbxHandle = Mailbox_handle(&mbxStruct);

    // Configure state machine and operating system objects
    // Completed, Failed, Cancelled
    Define_Global_States();

    // Main State Machine
    Define_State_Machine();

}

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                   Helper Functions
 * ----------------------------------------------------------------------------------------------------------------- */
void State_Machine_Thread_Start(UArg arg0, UArg arg1)
{
  State_Information_Ptr_t newStateInformation;
  MsgObj msg;
  uint16_t state_machine_transition_message_id;

  while ( 1 )
  {
    // Wait forever until we receive a flag
    if (
        g_ptr_Current_State_Info->stateNumber >= STATE_TOTAL_COUNT &&
        g_ptr_Current_State_Info->stateNumber != g_Completed_State_info.stateNumber &&
        g_ptr_Current_State_Info->stateNumber != g_Failed_State_info.stateNumber &&
        g_ptr_Current_State_Info->stateNumber != g_Cancelled_State_info.stateNumber)
    {
        System_printf( "State %u higher than STATE_TOTAL_COUNT: %u", g_ptr_Current_State_Info->stateNumber, STATE_TOTAL_COUNT);
    }
    else
    {
        System_printf( "State %u: %s waiting\n",
          g_ptr_Current_State_Info->stateNumber,
          state_to_string_table[g_ptr_Current_State_Info->stateNumber]);
    }

    Mailbox_pend(mbxHandle, &msg, BIOS_WAIT_FOREVER);

    state_machine_transition_message_id = msg.id;

    switch( state_machine_transition_message_id )
    {
      default:
        break;
    }

    // Use the event_flag as a key in the state transition table to find the next state
    newStateInformation = g_ptr_Current_State_Info->stateTransitionTable[ state_machine_transition_message_id ];

    // A NULL Ptr means that the event does not lead to a state transition.
    if (newStateInformation == NULL)
    {
      // Call Service_Null_Event to execute generic behavior that occurs whenever we receive an invalid event
      Service_Null_Message( state_machine_transition_message_id );
    }
    else
    {
      if(newStateInformation->stateExecutionFunction == NULL)
      {
          System_printf( "ERROR: NULL function pointer.\n");
      }
      else
      {
        // Call Leaving_Phase to execute generic behavior that occurs whenever we leave a phase
        Leaving_State( g_ptr_Current_State_Info );

        // Update the current phase information
        newStateInformation->previousStateInfo_ptr = g_ptr_Current_State_Info;
        g_ptr_Current_State_Info = newStateInformation;

        // Call Entering_Phase to execute generic behavior that occurs whenever we enter a new phase
        Entering_State( g_ptr_Current_State_Info );

        // Enter the next phase
        g_ptr_Current_State_Info->stateExecutionFunction( g_ptr_Current_State_Info );
      }
    }
  }
}

void Define_State_Machines( void )
{
}

