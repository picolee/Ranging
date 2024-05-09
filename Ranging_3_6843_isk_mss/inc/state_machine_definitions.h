/*
 * state_machine_definitions.h
 *
 *  Created on: Nov 7, 2023
 *      Author: Lee Lemay
 */

#ifndef STATE_MACHINE_DEFINITIONS_H_
#define STATE_MACHINE_DEFINITIONS_H_

#define  STATE_NUMBER_COMPLETE   100
#define  STATE_NUMBER_FAIL       200
#define  STATE_NUMBER_CANCELLED  300


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

Mailbox_Struct mbxStruct;
Mailbox_Handle mbxHandle;

// Forward declaration of State_Information_t
typedef struct State_Information_t State_Information_t;

// Define a type alias for a pointer to the struct
typedef struct State_Information_t* State_Information_Ptr_t;

// Function prototype for state transition functions
typedef void (*StateExecutionFunction_t)(State_Information_Ptr_t ptr_state_information);


/* Active Task Type */
// Keep in synch with robot_task_strings defined in Robot.c
typedef enum
{
    STATE_INIT              = 0U,
    STATE_STANDBY,
    STATE_CONFIGURE_RECEIVE,
    STATE_CONFIGURE_TRANSMIT,
    STATE_TRANSMIT_START_CODE,
    STATE_RECEIVE_START_CODE,
    STATE_TRANSMIT_RESPONSE_CODE,
    STATE_RECEIVE_RESPONSE_CODE,
    STATE_TOTAL_COUNT
} State_Enum_t;

typedef enum
{
  STATE_MACHINE_MSG_GO_STANDBY,
  STATE_MACHINE_MSG_CANCELLED,
  STATE_MACHINE_MSG_FAILED,
  STATE_MACHINE_MSG_COMPLETED,
  STATE_MACHINE_MSG_TOTAL_COUNT
}StateMachineMessages_t;

// Each State is represented by a State_Information_t struct.
// It contains:
// a pointer to the function that executes the phase.
// the transition table that maps a message that is received to the State_Information_Ptr_t that defines the new state.
// state information
// prior state information
struct State_Information_t
{
    State_Enum_t                    currentState;              /* state that corresponds to the state information struct */
    State_Information_Ptr_t         stateTransitionTable[STATE_MACHINE_MSG_TOTAL_COUNT]; /* Mapping between flags received and the new state to transition to */
    StateExecutionFunction_t        stateExecutionFunction;     /* pointer to the function that executes this state */
    State_Information_Ptr_t         previousStateInfo_ptr;      /* prior state information struct */
    uint16_t                        stateNumber;                /* The index of this phase. States are zero indexed and numbered in order of execution. */
    uint16_t                        prn;                        /* Gold Code PRN */
    uint8_t                         goldCodeNumBits;            /* 2^N + 1 possible PRNs, each of length 2^N-1 */
};
void Configure_Initial_State_To_Idle( void );
void Define_Global_States( void );
void Define_State_Machine( void );



/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Globally Exported Variables
 * ----------------------------------------------------------------------------------------------------------------- */

extern State_Information_Ptr_t  g_ptr_Current_Phase_Info;
extern State_Information_t      g_Completed_State_info;
extern State_Information_t      g_Failed_State_info;
extern State_Information_t      g_Cancelled_State_info;


#endif /* STATE_MACHINE_DEFINITIONS_H_ */
