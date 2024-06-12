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
#include <shared/ranging_rfConfig.h>

const char *  message_to_string_table[SM_MSG_TOTAL_COUNT] = {0};
const char *  state_to_string_table[STATE_TOTAL_COUNT] = {0};

StateMachine_t State_Machine;


// This array holds the State_Information_t that define the behavior of each state
State_Information_t State_Machine_States[STATE_TOTAL_COUNT]     = {0};

void Define_State_Machine( void )
{
    // Configure the string table
    uint16_t index;
    for(index = 0; index < SM_MSG_TOTAL_COUNT; index++)
    {
        message_to_string_table[index] = "MSG_UNINIT";
    }
    message_to_string_table[SM_MSG_INIT]                = "MSG_GO_INIT";
    message_to_string_table[SM_MSG_STANDBY]             = "MSG_GO_STANDBY";
    message_to_string_table[SM_MSG_BEGIN_RANGING]       = "MSG_BEGIN_RANGING";
    message_to_string_table[SM_MSG_SENSOR_STARTED]      = "MSG_SENSOR_STARTED";
    message_to_string_table[SM_MSG_TIMESLOT_STARTED]    = "MSG_SLOT_STARTED";
    message_to_string_table[SM_MSG_CFG_NEXT_TIMESLOT]   = "MSG_CFG_NEXT_TIMESLOT";
    message_to_string_table[SM_MSG_START_EXECUTING]     = "MSG_START_EXECUTING";
    message_to_string_table[SM_MSG_RESULTS_AVAIL]       = "MSG_RESULTS_AVAIL";
    message_to_string_table[SM_MSG_CANCELLED]           = "MSG_CANCELLED";
    message_to_string_table[SM_MSG_FAILED]              = "MSG_FAILED";
    message_to_string_table[SM_MSG_COMPLETED]           = "MSG_COMPLETED";
    message_to_string_table[SM_MSG_DSS_REPORTS_FAILURE] = "MSG_DSS_FAILURE";
    message_to_string_table[SM_MSG_UPDATE_TIMESLOT_LIST]= "MSG_UPDATE_SLOTS";

    for(index = 0; index < STATE_TOTAL_COUNT; index++)
    {
        state_to_string_table[index] = "STATE_UNINIT";
    }
    state_to_string_table[STATE_INIT]               = "INIT\t";
    state_to_string_table[STATE_STANDBY]            = "STANDBY\t";
    state_to_string_table[STATE_UPDATE_TIMESLOTS]   = "UPDATE_SLOTS";
    state_to_string_table[STATE_CFG]                = "CFG\t";
    state_to_string_table[STATE_ACTIVATE_CFG]       = "ACTIVATE_CFG";
    state_to_string_table[STATE_EXECUTE_CFG]        = "START_EXECUTE";
    state_to_string_table[STATE_EXECUTING]          = "EXECUTING";
    state_to_string_table[STATE_PROCESS_RESULT]     = "PROCESS_RESULT";
    state_to_string_table[STATE_COMPLETED]          = "STATE_COMPLETED";
    state_to_string_table[STATE_FAILED]             = "STATE_FAILED";
    state_to_string_table[STATE_CANCELLED]          = "STATE_CANCELLED";

    // Initialization
    State_Machine_States[STATE_INIT].stateMachine                             = &State_Machine;
    State_Machine_States[STATE_INIT].stateExecutionFunction                   = SM_Func_Initialization;
    State_Machine_States[STATE_INIT].stateNumber                              = STATE_INIT;
    State_Machine_States[STATE_INIT].previousStateInfo_ptr                    = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_INIT].stateTransitionTable[SM_MSG_INIT]        = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_INIT].stateTransitionTable[SM_MSG_STANDBY]     = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_INIT].stateTransitionTable[SM_MSG_COMPLETED]   = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_INIT].stateTransitionTable[SM_MSG_FAILED]      = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_INIT].stateTransitionTable[SM_MSG_CANCELLED]   = &State_Machine_States[STATE_CANCELLED];

    // Standby
    State_Machine_States[STATE_STANDBY].stateMachine                                        = &State_Machine;
    State_Machine_States[STATE_STANDBY].stateExecutionFunction                              = SM_Func_Standby;
    State_Machine_States[STATE_STANDBY].stateNumber                                         = STATE_STANDBY;
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_INIT]                   = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_STANDBY]                = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_FAILED]                 = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_CANCELLED]              = &State_Machine_States[STATE_CANCELLED];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_BEGIN_RANGING]          = &State_Machine_States[STATE_CFG];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_UPDATE_TIMESLOT_LIST]   = &State_Machine_States[STATE_UPDATE_TIMESLOTS];

    // Update time slots
    State_Machine_States[STATE_UPDATE_TIMESLOTS].stateMachine                           = &State_Machine;
    State_Machine_States[STATE_UPDATE_TIMESLOTS].stateExecutionFunction                 = SM_Func_Update_List_Of_Timeslots;
    State_Machine_States[STATE_UPDATE_TIMESLOTS].stateNumber                            = STATE_UPDATE_TIMESLOTS;
    State_Machine_States[STATE_UPDATE_TIMESLOTS].stateTransitionTable[SM_MSG_INIT]      = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_UPDATE_TIMESLOTS].stateTransitionTable[SM_MSG_STANDBY]   = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_UPDATE_TIMESLOTS].stateTransitionTable[SM_MSG_FAILED]    = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_UPDATE_TIMESLOTS].stateTransitionTable[SM_MSG_CANCELLED] = &State_Machine_States[STATE_CANCELLED];
    State_Machine_States[STATE_UPDATE_TIMESLOTS].stateTransitionTable[SM_MSG_COMPLETED] = &State_Machine_States[STATE_STANDBY];

    // Configure
    State_Machine_States[STATE_CFG].stateMachine                            = &State_Machine;
    State_Machine_States[STATE_CFG].stateExecutionFunction                  = SM_Func_Cfg;
    State_Machine_States[STATE_CFG].stateNumber                             = STATE_CFG;
    State_Machine_States[STATE_CFG].stateTransitionTable[SM_MSG_INIT]       = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_CFG].stateTransitionTable[SM_MSG_STANDBY]    = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_CFG].stateTransitionTable[SM_MSG_FAILED]     = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_CFG].stateTransitionTable[SM_MSG_CANCELLED]  = &State_Machine_States[STATE_CANCELLED];
    State_Machine_States[STATE_CFG].stateTransitionTable[SM_MSG_COMPLETED]  = &State_Machine_States[STATE_ACTIVATE_CFG];

    // Activate CFG
    State_Machine_States[STATE_ACTIVATE_CFG].stateMachine                               = &State_Machine;
    State_Machine_States[STATE_ACTIVATE_CFG].stateExecutionFunction                     = SM_Func_Activate_Cfg;
    State_Machine_States[STATE_ACTIVATE_CFG].stateNumber                                = STATE_ACTIVATE_CFG;
    State_Machine_States[STATE_ACTIVATE_CFG].stateTransitionTable[SM_MSG_INIT]          = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_ACTIVATE_CFG].stateTransitionTable[SM_MSG_STANDBY]       = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_ACTIVATE_CFG].stateTransitionTable[SM_MSG_FAILED]        = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_ACTIVATE_CFG].stateTransitionTable[SM_MSG_CANCELLED]     = &State_Machine_States[STATE_CANCELLED];
    State_Machine_States[STATE_ACTIVATE_CFG].stateTransitionTable[SM_MSG_COMPLETED]     = &State_Machine_States[STATE_EXECUTE_CFG];

    // EXECUTE
    State_Machine_States[STATE_EXECUTE_CFG].stateMachine                                    = &State_Machine;
    State_Machine_States[STATE_EXECUTE_CFG].stateExecutionFunction                          = SM_Func_Execute_Cfg;
    State_Machine_States[STATE_EXECUTE_CFG].stateNumber                                     = STATE_EXECUTE_CFG;
    State_Machine_States[STATE_EXECUTE_CFG].stateTransitionTable[SM_MSG_INIT]               = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_EXECUTE_CFG].stateTransitionTable[SM_MSG_STANDBY]            = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_EXECUTE_CFG].stateTransitionTable[SM_MSG_FAILED]             = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_EXECUTE_CFG].stateTransitionTable[SM_MSG_CANCELLED]          = &State_Machine_States[STATE_CANCELLED];
    State_Machine_States[STATE_EXECUTE_CFG].stateTransitionTable[SM_MSG_SENSOR_STARTED]     = &State_Machine_States[STATE_EXECUTING];
    State_Machine_States[STATE_EXECUTE_CFG].stateTransitionTable[SM_MSG_TIMESLOT_STARTED]   = &State_Machine_States[STATE_EXECUTING];   // NO-OP

    // EXECUTING
    State_Machine_States[STATE_EXECUTING].stateMachine                                      = &State_Machine;
    State_Machine_States[STATE_EXECUTING].stateExecutionFunction                            = SM_Func_Executing;
    State_Machine_States[STATE_EXECUTING].stateNumber                                       = STATE_EXECUTING;
    State_Machine_States[STATE_EXECUTING].stateTransitionTable[SM_MSG_INIT]                 = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_EXECUTING].stateTransitionTable[SM_MSG_STANDBY]              = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_EXECUTING].stateTransitionTable[SM_MSG_FAILED]               = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_EXECUTING].stateTransitionTable[SM_MSG_CANCELLED]            = &State_Machine_States[STATE_CANCELLED];
    State_Machine_States[STATE_EXECUTING].stateTransitionTable[SM_MSG_RESULTS_AVAIL]        = &State_Machine_States[STATE_PROCESS_RESULT];  // Results from sensor, stop the sensor then process
    State_Machine_States[STATE_EXECUTING].stateTransitionTable[SM_MSG_CFG_NEXT_TIMESLOT]    = &State_Machine_States[STATE_CFG];             // If this state is a NO-OP

    // Results Available
    State_Machine_States[STATE_PROCESS_RESULT].stateMachine                                     = &State_Machine;
    State_Machine_States[STATE_PROCESS_RESULT].stateExecutionFunction                           = SM_Func_Process_Result;
    State_Machine_States[STATE_PROCESS_RESULT].stateNumber                                      = STATE_PROCESS_RESULT;
    State_Machine_States[STATE_PROCESS_RESULT].stateTransitionTable[SM_MSG_INIT]                = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_PROCESS_RESULT].stateTransitionTable[SM_MSG_STANDBY]             = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_PROCESS_RESULT].stateTransitionTable[SM_MSG_FAILED]              = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_PROCESS_RESULT].stateTransitionTable[SM_MSG_CANCELLED]           = &State_Machine_States[STATE_CANCELLED];
    State_Machine_States[STATE_PROCESS_RESULT].stateTransitionTable[SM_MSG_RESULTS_AVAIL]       = &State_Machine_States[STATE_PROCESS_RESULT];
    State_Machine_States[STATE_PROCESS_RESULT].stateTransitionTable[SM_MSG_CFG_NEXT_TIMESLOT]   = &State_Machine_States[STATE_CFG];
    State_Machine_States[STATE_PROCESS_RESULT].stateTransitionTable[SM_MSG_START_EXECUTING]     = &State_Machine_States[STATE_EXECUTE_CFG];

    // Completed
    State_Machine_States[STATE_COMPLETED].stateMachine                                  = &State_Machine;
    State_Machine_States[STATE_COMPLETED].stateExecutionFunction                        = SM_Func_Task_Completed_Successfully;
    State_Machine_States[STATE_COMPLETED].stateTransitionTable[SM_MSG_STANDBY]          = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_COMPLETED].stateTransitionTable[SM_MSG_COMPLETED]        = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_COMPLETED].stateNumber                                   = STATE_COMPLETED;

    // Failed
    State_Machine_States[STATE_FAILED].stateMachine                                     = &State_Machine;
    State_Machine_States[STATE_FAILED].stateExecutionFunction                           = SM_Func_Task_Failed;
    State_Machine_States[STATE_FAILED].stateTransitionTable[SM_MSG_STANDBY]             = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_FAILED].stateTransitionTable[SM_MSG_COMPLETED]           = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_FAILED].stateNumber                                      = STATE_FAILED;

    // Cancelled
    State_Machine_States[STATE_CANCELLED].stateMachine                                  = &State_Machine;
    State_Machine_States[STATE_CANCELLED].stateExecutionFunction                        = SM_Func_Task_Cancelled;
    State_Machine_States[STATE_CANCELLED].stateTransitionTable[SM_MSG_STANDBY]          = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_CANCELLED].stateNumber                                   = STATE_CANCELLED;
}
