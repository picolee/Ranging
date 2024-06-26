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
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Global Variables
 * ----------------------------------------------------------------------------------------------------------------- */

extern const char *  message_to_string_table[SM_MSG_TOTAL_COUNT];
extern const char *  state_to_string_table[STATE_TOTAL_COUNT];

extern State_Information_Ptr_t State_Machine_States;
extern StateMachine_t State_Machine;


/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                   Local Variable Declarations
 * ----------------------------------------------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                   Local Function Declarations
 * ----------------------------------------------------------------------------------------------------------------- */

void State_Machine_Entry_Point(StateMachine_t *p_stateMachine,
                               State_Information_t *p_state_array,
                               uint16_t totalStates,
                               uint8_t taskPriority,
                               UART_Handle uart);
void State_Machine_Thread_Start(UArg arg0, UArg arg1);

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                   External Functions
 * ----------------------------------------------------------------------------------------------------------------- */

//void State_Machine_Init(uint8_t taskPriority, Task_Handle* stateMachineTask)
//{
//
//    // Create a Mailbox instance */
//    Mailbox_Params mbxParams;
//    Mailbox_Params_init(&mbxParams);
//    mbxParams.buf     = (Ptr)mailboxBuffer;
//    mbxParams.bufSize = sizeof(mailboxBuffer);
//    Mailbox_construct(&mbxStruct, sizeof(MsgObj), NUMMSGS, &mbxParams, NULL);
//    mbxHandle = Mailbox_handle(&mbxStruct);
//
//    // Configure state machine and operating system objects
//    // Completed, Failed, Cancelled
//    Define_Global_States();
//
//    // Main State Machine
//    Define_State_Machine();
//
//    Configure_Initial_State_To_Standby();
//
//    // Create thread
//    Task_Params         taskParams;
//    Task_Params_init(&taskParams);
//    taskParams.priority  = taskPriority;
//    taskParams.stackSize = 4*1024;
//    *stateMachineTask = Task_create(State_Machine_Thread_Start, &taskParams, NULL);
//
//    Send_State_Machine_Init_Message();
//}

void State_Machine_Init(uint8_t taskPriority, UART_Handle uart)
{
    // Define all of the states and string tables for the main state machine
    Define_State_Machine();

    // Initialize the state machine mailboxes and thread
    State_Machine_Entry_Point(&State_Machine, &(State_Machine_States[0]), STATE_TOTAL_COUNT, taskPriority, uart);

    Send_State_Machine_Init_Message();
}

// This is a generalized entry point that allows creation of multiple state machines, each running in their own thread
void State_Machine_Entry_Point(StateMachine_Ptr_t p_stateMachine,
                               State_Information_Ptr_t p_state_array,
                               uint16_t totalStates,
                               uint8_t taskPriority,
                               UART_Handle uart)
{
    p_stateMachine->states = p_state_array;
    p_stateMachine->totalStates = totalStates;
    p_stateMachine->currentState = &p_state_array[0]; // Assuming the first state is the initial state
    p_stateMachine->uartHandle = uart;

    Mailbox_Params mbxParams;
    Mailbox_Params_init(&mbxParams);
    mbxParams.buf = (Ptr)mailboxBuffer;
    mbxParams.bufSize = sizeof(mailboxBuffer);
    Mailbox_construct(&p_stateMachine->mbxStruct, sizeof(MsgObj), NUMMSGS, &mbxParams, NULL);
    p_stateMachine->mbxHandle = Mailbox_handle(&p_stateMachine->mbxStruct);

    Task_Params_init(&p_stateMachine->taskParams);
    p_stateMachine->taskParams.priority = taskPriority;
    p_stateMachine->taskParams.stackSize = 4 * 1024;
    p_stateMachine->taskParams.arg0 = (UArg)p_stateMachine;
    p_stateMachine->taskHandle = Task_create(State_Machine_Thread_Start, &p_stateMachine->taskParams, NULL);
}



/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                   Helper Functions
 * ----------------------------------------------------------------------------------------------------------------- */
void State_Machine_Thread_Start(UArg arg0, UArg arg1)
{
    StateMachine_t *sm = (StateMachine_t *)arg0;
    State_Information_Ptr_t newStateInformation;
    MsgObj msg;
    uint16_t state_machine_transition_message_id;
    uint32_t timeTSCL;

    while ( 1 )
    {
        timeTSCL = Cycleprofiler_getTimeStamp();

        // Wait forever until we receive a flag
        if ( sm->currentState->stateNumber >= STATE_TOTAL_COUNT )
        {
            System_printf( "State %u higher than STATE_TOTAL_COUNT: %u", sm->currentState->stateNumber, STATE_TOTAL_COUNT);
        }
        else
        {
//            System_printf( "%s wait: %u\n",
//                           state_to_string_table[sm->currentState->stateNumber],
//                           timeTSCL);
//            CLI_write ("%s wait: %u\n",
//                       state_to_string_table[sm->currentState->stateNumber],
//                       timeTSCL);
        }

        Mailbox_pend(State_Machine.mbxHandle, &msg, BIOS_WAIT_FOREVER);

        state_machine_transition_message_id = msg.id;

        switch( state_machine_transition_message_id )
        {
          default:
            break;
        }

        // Use the event_flag as a key in the state transition table to find the next state
        newStateInformation = sm->currentState->stateTransitionTable[ state_machine_transition_message_id ];

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
                Leaving_State( sm->currentState );

                // Update the current phase information
                newStateInformation->previousStateInfo_ptr = sm->currentState;
                sm->currentState = newStateInformation;

                // Call Entering_Phase to execute generic behavior that occurs whenever we enter a new phase
                Entering_State( sm->currentState );

                // Enter the next phase
                sm->currentState->stateExecutionFunction( sm->currentState );
            }
        }
    }
}

void Begin_State_Machine_Receive_Start_Code()
{
    Send_State_Machine_Message( SM_MSG_RX_START_CODE );
}

void Configure_State_Machine_Receive( float frequencyInGhz, uint16_t prn )
{
    uint16_t index;
    for(index = 0; index < STATE_TOTAL_COUNT; index++)
    {
        State_Machine_States[index].rxPrn = prn;
        State_Machine_States[index].rxFrequencyInGhz = frequencyInGhz;
    }
    Send_State_Machine_Message( SM_MSG_CFG_RX );
}

void Activate_State_Machine_Receive_Cfg( )
{
    Send_State_Machine_Message( SM_MSG_ACTIVATE_RX_CFG );
}

void Configure_State_Machine_Transmit( float frequencyInGhz, uint16_t prn )
{
    uint16_t index;
    for(index = 0; index < STATE_TOTAL_COUNT; index++)
    {
        State_Machine_States[index].txPrn = prn;
        State_Machine_States[index].txFrequencyInGhz = frequencyInGhz;
    }
    Send_State_Machine_Message( SM_MSG_CFG_TX );
}

void Activate_State_Machine_Transmit_Cfg()
{
    Send_State_Machine_Message( SM_MSG_ACTIVATE_TX_CFG );
}

void Begin_State_Machine_Transmit_Start_Code()
{
    Send_State_Machine_Message( SM_MSG_TX_START_CODE );
}

void Begin_State_Machine_Transmit_Response_Code()
{
    Send_State_Machine_Message( SM_MSG_TX_RESPONSE_CODE );
}

void Send_State_Machine_Standby_Message( )
{
    Send_State_Machine_Message( SM_MSG_STANDBY );
}

void Send_State_Machine_Init_Message( )
{
    Send_State_Machine_Message( SM_MSG_INIT );
}

void Send_Code_Detect_Message( )
{
    Send_State_Machine_Message( SM_MSG_CODE_DETECT );
}

void Send_No_Code_Detect_Message( )
{
    Send_State_Machine_Message( SM_MSG_NO_CODE_DETECT );
}

void Send_Transmit_Complete_Message()
{
    Send_State_Machine_Message( SM_MSG_TX_COMPLETE );
}
