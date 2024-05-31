/*
 * state_machine_definitions.h
 *
 *  Created on: Nov 7, 2023
 *      Author: Lee Lemay
 */


/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                     DEFINES
 * ----------------------------------------------------------------------------------------------------------------- */

#ifndef STATE_MACHINE_DEFINITIONS_H_
#define STATE_MACHINE_DEFINITIONS_H_

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                     Includes
 * ----------------------------------------------------------------------------------------------------------------- */

#include <stdint.h>


/* XDC module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/drivers/uart/UART.h>

#include <inc/ranging_common.h>
#include <shared/ranging_timeslot.h>

#define NUMMSGS 10

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                     Typedefs
 * ----------------------------------------------------------------------------------------------------------------- */

/*
 * This type is accessed by the application. When changing the data members of
 * this structure, considerations should be made for padding and data alignment.
 */
typedef struct MsgObj
{
    uint16_t id;
} MsgObj;

/*
 * Mailbox messages are stored in a queue that requires a header in front of
 * each message. Mailbox_MbxElem is defined such that the header and its size
 * are factored into the total data size requirement for a mailbox instance.
 * Because Mailbox_MbxElem contains Int data types, padding may be added to
 * this struct depending on the data members defined in MsgObj.
 */
typedef struct MailboxMsgObj
{
    Mailbox_MbxElem elem; /* Mailbox header        */
    MsgObj obj;           /* Application's mailbox */
} MailboxMsgObj;


/* This buffer is not directly accessed by the application */
MailboxMsgObj mailboxBuffer[NUMMSGS];

//Mailbox_Struct mbxStruct;
//Mailbox_Handle mbxHandle;

// Forward declaration of State_Information_t
typedef struct State_Information_t State_Information_t;

// Define a type alias for a pointer to the struct
typedef struct State_Information_t* State_Information_Ptr_t;

// Forward declaration of StateMachine_t
typedef struct StateMachine_t StateMachine_t;

// Define a type alias for a pointer to the struct
typedef struct StateMachine_t* StateMachine_Ptr_t;

// Function prototype for state transition functions
typedef void (*StateExecutionFunction_t)(State_Information_Ptr_t ptr_state_information);


/* Active Task Type */
// Keep in synch with robot_task_strings defined in Robot.c
typedef enum
{
    STATE_INIT              = 0U,
    STATE_STANDBY,
    STATE_UPDATE_TIMESLOTS,
    STATE_CFG,
    STATE_ACTIVATE_CFG,
    STATE_EXECUTE_CFG,
    STATE_EXECUTING,
    STATE_STOP_EXECUTION,
    STATE_PROCESS_RESULT,
    STATE_COMPLETED,
    STATE_FAILED,
    STATE_CANCELLED,
    STATE_TOTAL_COUNT
} State_Enum_t;

typedef enum
{
    SM_MSG_INIT       = 0U,
    SM_MSG_STANDBY,
    SM_MSG_BEGIN_RANGING,
    SM_MSG_SENSOR_STARTED,
    SM_MSG_TIMESLOT_STARTED,
    SM_MSG_RESULTS_AVAIL,
    SM_MSG_CFG_NEXT_TIMESLOT,
    SM_MSG_DSS_REPORTS_FAILURE,
    SM_MSG_UPDATE_TIMESLOT_LIST,
    SM_MSG_CANCELLED,
    SM_MSG_FAILED,
    SM_MSG_COMPLETED,
    SM_MSG_TOTAL_COUNT
} StateMachineMessages_t;

// Each State is represented by a State_Information_t struct.
// It contains:
// a pointer to the function that executes the phase.
// the transition table that maps a message that is received to the State_Information_Ptr_t that defines the new state.
struct State_Information_t
{
    State_Information_Ptr_t         stateTransitionTable[SM_MSG_TOTAL_COUNT]; // Mapping between flags received and the new state to transition to
    StateExecutionFunction_t        stateExecutionFunction;     // Pointer to the function that executes this state
    State_Information_Ptr_t         previousStateInfo_ptr;      // Pointer to the prior state information struct
    uint16_t                        stateNumber;                // State_Enum_t that corresponds to the state information struct
    uint32_t                        timesEntered;               // Tracks the total number of times this state was entered
    StateMachine_Ptr_t              stateMachine;               // Pointer to the owning state machine
    rangingTimeSlot_Ptr_t           previousTimeSlot_ptr;       // Pointer to the previous time slot
    rangingTimeSlot_Ptr_t           currentTimeSlot_ptr;        // Pointer to the current time slot
    rangingTimeSlot_Ptr_t           nextTimeSlot_ptr;           // Pointer to the next time slot
};

struct StateMachine_t
{
    State_Information_Ptr_t currentState;
    State_Information_t *states;
    uint16_t totalStates;
    Mailbox_Handle mbxHandle;
    Mailbox_Struct mbxStruct;
    Task_Handle taskHandle;
    Task_Params taskParams;
    UART_Handle uartHandle;
};

void Configure_Initial_State_To_Idle( void );
void Define_Global_States( void );
void Define_State_Machine( void );



/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Globally Exported Variables
 * ----------------------------------------------------------------------------------------------------------------- */



#endif /* STATE_MACHINE_DEFINITIONS_H_ */
