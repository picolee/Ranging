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
        message_to_string_table[index] = "UNINITIALIZED";
    }
    message_to_string_table[SM_MSG_INIT]                = "MSG_GO_INIT";
    message_to_string_table[SM_MSG_STANDBY]             = "MSG_GO_STANDBY";
    message_to_string_table[SM_MSG_CFG_RX]              = "MSG_GO_CFG_RX";
    message_to_string_table[SM_MSG_CFG_TX]              = "SM_MSG_CFG_TX";
    message_to_string_table[SM_MSG_ACTIVATE_RX_CFG]     = "SM_MSG_ACTIVATE_RX_CFG";
    message_to_string_table[SM_MSG_ACTIVATE_TX_CFG]     = "SM_MSG_ACTIVATE_TX_CFG";
    message_to_string_table[SM_MSG_RX_START_CODE]       = "MSG_GO_RX";
    message_to_string_table[SM_MSG_RX_RESPONSE_CODE]    = "SM_MSG_RX_RESPONSE_CODE";
    message_to_string_table[SM_MSG_TX_START_CODE]       = "SM_MSG_TX_START_CODE";
    message_to_string_table[SM_MSG_TX_RESPONSE_CODE]    = "SM_MSG_TX_RESPONSE_CODE";
    message_to_string_table[SM_MSG_CODE_DETECT]         = "SM_MSG_CODE_DETECT";
    message_to_string_table[SM_MSG_NO_CODE_DETECT]      = "SM_MSG_NO_CODE_DETECT";
    message_to_string_table[SM_MSG_TX_COMPLETE]         = "SM_MSG_TX_COMPLETE";
    message_to_string_table[SM_MSG_CANCELLED]           = "MSG_CANCELLED";
    message_to_string_table[SM_MSG_FAILED]              = "MSG_FAILED";
    message_to_string_table[SM_MSG_COMPLETED]           = "MSG_COMPLETED";
    message_to_string_table[SM_MSG_DSS_REPORTS_SUCCESS] = "DSS_SUCCESS";
    message_to_string_table[SM_MSG_DSS_REPORTS_FAILURE] = "DSS_FAILURE";
    message_to_string_table[SM_MSG_SENSOR_STARTED]      = "SENSOR_STARTED";

    for(index = 0; index < STATE_TOTAL_COUNT; index++)
    {
        state_to_string_table[index] = "UNINITIALIZED";
    }
    state_to_string_table[STATE_INIT]             = "INIT";
    state_to_string_table[STATE_STANDBY]          = "STANDBY";
    state_to_string_table[STATE_CFG_RX]           = "CFG_RX";
    state_to_string_table[STATE_CFG_TX]           = "CFG_TX";
    state_to_string_table[STATE_ACTIVATE_RX_CFG]  = "ACTIVATE_RX_CFG";
    state_to_string_table[STATE_ACTIVATE_TX_CFG]  = "ACTIVATE_TX_CFG";
    state_to_string_table[STATE_TX_START_CODE]    = "TX_START_CODE";
    state_to_string_table[STATE_RX_START_CODE]    = "RX_START_CODE";
    state_to_string_table[STATE_TX_RESPONSE_CODE] = "TX_RSP_CODE";
    state_to_string_table[STATE_RX_RESPONSE_CODE] = "RX_RSP_CODE";
    state_to_string_table[STATE_RX_CONTINUE]      = "STATE_RX_CONTINUE";
    state_to_string_table[STATE_COMPLETED]        = "STATE_COMPLETED";
    state_to_string_table[STATE_FAILED]           = "STATE_FAILED";
    state_to_string_table[STATE_CANCELLED]        = "STATE_CANCELLED";

    // Initialization
    State_Machine_States[STATE_INIT].stateMachine                             = &State_Machine;
    State_Machine_States[STATE_INIT].stateExecutionFunction                   = SM_Func_Initialization;
    State_Machine_States[STATE_INIT].stateNumber                              = STATE_INIT;
    State_Machine_States[STATE_INIT].rxFrequencyInGhz                         = RX_FREQUENCY_GHZ;
    State_Machine_States[STATE_INIT].txFrequencyInGhz                         = TX_FREQUENCY_GHZ;
    State_Machine_States[STATE_INIT].txPrn                                    = DEFAULT_PRN;
    State_Machine_States[STATE_INIT].rxPrn                                    = DEFAULT_PRN;
    State_Machine_States[STATE_INIT].goldCodeNumBits                          = GOLD_CODE_NUM_BITS;
    State_Machine_States[STATE_INIT].previousStateInfo_ptr                    = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_INIT].stateTransitionTable[SM_MSG_INIT]        = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_INIT].stateTransitionTable[SM_MSG_STANDBY]     = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_INIT].stateTransitionTable[SM_MSG_COMPLETED]   = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_INIT].stateTransitionTable[SM_MSG_FAILED]      = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_INIT].stateTransitionTable[SM_MSG_CANCELLED]   = &State_Machine_States[STATE_CANCELLED];

    // Standby
    State_Machine_States[STATE_STANDBY].stateMachine                                  = &State_Machine;
    State_Machine_States[STATE_STANDBY].stateExecutionFunction                        = SM_Func_Standby;
    State_Machine_States[STATE_STANDBY].stateNumber                                   = STATE_STANDBY;
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_INIT]             = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_STANDBY]          = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_CFG_RX]           = &State_Machine_States[STATE_CFG_RX];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_CFG_TX]           = &State_Machine_States[STATE_CFG_TX];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_ACTIVATE_RX_CFG]  = &State_Machine_States[STATE_ACTIVATE_RX_CFG];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_ACTIVATE_TX_CFG]  = &State_Machine_States[STATE_ACTIVATE_TX_CFG];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_TX_START_CODE]    = &State_Machine_States[STATE_TX_START_CODE];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_TX_RESPONSE_CODE] = &State_Machine_States[STATE_TX_RESPONSE_CODE];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_RX_START_CODE]    = &State_Machine_States[STATE_RX_START_CODE];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_RX_RESPONSE_CODE] = &State_Machine_States[STATE_RX_RESPONSE_CODE];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_FAILED]           = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_STANDBY].stateTransitionTable[SM_MSG_CANCELLED]        = &State_Machine_States[STATE_CANCELLED];

    // Configure TX
    State_Machine_States[STATE_CFG_TX].stateMachine                           = &State_Machine;
    State_Machine_States[STATE_CFG_TX].stateExecutionFunction                 = SM_Func_Cfg_Tx;
    State_Machine_States[STATE_CFG_TX].stateNumber                            = STATE_CFG_TX;
    State_Machine_States[STATE_CFG_TX].txFrequencyInGhz                       = TX_FREQUENCY_GHZ;
    State_Machine_States[STATE_CFG_TX].txPrn                                  = DEFAULT_PRN;
    State_Machine_States[STATE_CFG_TX].rxPrn                                  = DEFAULT_PRN;
    State_Machine_States[STATE_CFG_TX].goldCodeNumBits                        = GOLD_CODE_NUM_BITS;
    State_Machine_States[STATE_CFG_TX].stateTransitionTable[SM_MSG_INIT]      = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_CFG_TX].stateTransitionTable[SM_MSG_STANDBY]   = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_CFG_TX].stateTransitionTable[SM_MSG_FAILED]    = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_CFG_TX].stateTransitionTable[SM_MSG_CANCELLED] = &State_Machine_States[STATE_CANCELLED];

    // Activate TX CFG
    State_Machine_States[STATE_ACTIVATE_TX_CFG].stateMachine                              = &State_Machine;
    State_Machine_States[STATE_ACTIVATE_TX_CFG].stateExecutionFunction                    = SM_Func_Activate_Tx_Cfg;
    State_Machine_States[STATE_ACTIVATE_TX_CFG].stateNumber                               = STATE_ACTIVATE_TX_CFG;
    State_Machine_States[STATE_ACTIVATE_TX_CFG].txFrequencyInGhz                          = TX_FREQUENCY_GHZ;
    State_Machine_States[STATE_ACTIVATE_TX_CFG].txPrn                                     = DEFAULT_PRN;
    State_Machine_States[STATE_ACTIVATE_TX_CFG].rxPrn                                     = DEFAULT_PRN;
    State_Machine_States[STATE_ACTIVATE_TX_CFG].goldCodeNumBits                           = GOLD_CODE_NUM_BITS;
    State_Machine_States[STATE_ACTIVATE_TX_CFG].stateTransitionTable[SM_MSG_INIT]         = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_ACTIVATE_TX_CFG].stateTransitionTable[SM_MSG_STANDBY]      = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_ACTIVATE_TX_CFG].stateTransitionTable[SM_MSG_FAILED]       = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_ACTIVATE_TX_CFG].stateTransitionTable[SM_MSG_CANCELLED]    = &State_Machine_States[STATE_CANCELLED];

    // TX Start Code
    State_Machine_States[STATE_TX_START_CODE].stateMachine                                = &State_Machine;
    State_Machine_States[STATE_TX_START_CODE].stateExecutionFunction                      = SM_Func_Tx_Start_Code;
    State_Machine_States[STATE_TX_START_CODE].stateNumber                                 = STATE_TX_START_CODE;
    State_Machine_States[STATE_TX_START_CODE].txFrequencyInGhz                            = TX_FREQUENCY_GHZ;
    State_Machine_States[STATE_TX_START_CODE].txPrn                                       = DEFAULT_PRN;
    State_Machine_States[STATE_TX_START_CODE].rxPrn                                       = DEFAULT_PRN;
    State_Machine_States[STATE_TX_START_CODE].goldCodeNumBits                             = GOLD_CODE_NUM_BITS;
    State_Machine_States[STATE_TX_START_CODE].stateTransitionTable[SM_MSG_INIT]           = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_TX_START_CODE].stateTransitionTable[SM_MSG_STANDBY]        = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_TX_START_CODE].stateTransitionTable[SM_MSG_TX_COMPLETE]    = &State_Machine_States[STATE_ACTIVATE_RX_CFG];
    State_Machine_States[STATE_TX_START_CODE].stateTransitionTable[SM_MSG_FAILED]         = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_TX_START_CODE].stateTransitionTable[SM_MSG_CANCELLED]      = &State_Machine_States[STATE_CANCELLED];

    // TX Response Code
    State_Machine_States[STATE_TX_RESPONSE_CODE].stateMachine                             = &State_Machine;
    State_Machine_States[STATE_TX_RESPONSE_CODE].stateExecutionFunction                   = SM_Func_Tx_Response_Code;
    State_Machine_States[STATE_TX_RESPONSE_CODE].stateNumber                              = STATE_TX_RESPONSE_CODE;
    State_Machine_States[STATE_TX_RESPONSE_CODE].txFrequencyInGhz                         = TX_FREQUENCY_GHZ;
    State_Machine_States[STATE_TX_RESPONSE_CODE].txPrn                                    = DEFAULT_PRN;
    State_Machine_States[STATE_TX_RESPONSE_CODE].rxPrn                                    = DEFAULT_PRN;
    State_Machine_States[STATE_TX_RESPONSE_CODE].goldCodeNumBits                          = GOLD_CODE_NUM_BITS;
    State_Machine_States[STATE_TX_RESPONSE_CODE].stateTransitionTable[SM_MSG_INIT]        = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_TX_RESPONSE_CODE].stateTransitionTable[SM_MSG_STANDBY]     = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_TX_RESPONSE_CODE].stateTransitionTable[SM_MSG_COMPLETED]   = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_TX_RESPONSE_CODE].stateTransitionTable[SM_MSG_TX_COMPLETE] = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_TX_RESPONSE_CODE].stateTransitionTable[SM_MSG_FAILED]      = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_TX_RESPONSE_CODE].stateTransitionTable[SM_MSG_CANCELLED]   = &State_Machine_States[STATE_CANCELLED];

    // Configure RX
    State_Machine_States[STATE_CFG_RX].stateMachine                                   = &State_Machine;
    State_Machine_States[STATE_CFG_RX].stateExecutionFunction                         = SM_Func_Cfg_Rx;
    State_Machine_States[STATE_CFG_RX].stateNumber                                    = STATE_CFG_RX;
    State_Machine_States[STATE_CFG_RX].txPrn                                          = DEFAULT_PRN;
    State_Machine_States[STATE_CFG_RX].rxPrn                                          = DEFAULT_PRN;
    State_Machine_States[STATE_CFG_RX].goldCodeNumBits                                = GOLD_CODE_NUM_BITS;
    State_Machine_States[STATE_CFG_RX].rxFrequencyInGhz                               = RX_FREQUENCY_GHZ;
    State_Machine_States[STATE_CFG_RX].stateTransitionTable[SM_MSG_INIT]              = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_CFG_RX].stateTransitionTable[SM_MSG_STANDBY]           = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_CFG_RX].stateTransitionTable[SM_MSG_RX_START_CODE]     = &State_Machine_States[STATE_RX_START_CODE];
    State_Machine_States[STATE_CFG_RX].stateTransitionTable[SM_MSG_RX_RESPONSE_CODE]  = &State_Machine_States[STATE_RX_RESPONSE_CODE];
    State_Machine_States[STATE_CFG_RX].stateTransitionTable[SM_MSG_ACTIVATE_RX_CFG]   = &State_Machine_States[STATE_ACTIVATE_RX_CFG];
    State_Machine_States[STATE_CFG_RX].stateTransitionTable[SM_MSG_FAILED]            = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_CFG_RX].stateTransitionTable[SM_MSG_CANCELLED]         = &State_Machine_States[STATE_CANCELLED];

    // Activate Rx CFG
    State_Machine_States[STATE_ACTIVATE_RX_CFG].stateMachine                                    = &State_Machine;
    State_Machine_States[STATE_ACTIVATE_RX_CFG].stateExecutionFunction                          = SM_Func_Activate_Rx_Cfg;
    State_Machine_States[STATE_ACTIVATE_RX_CFG].stateNumber                                     = STATE_ACTIVATE_RX_CFG;
    State_Machine_States[STATE_ACTIVATE_RX_CFG].rxFrequencyInGhz                                = RX_FREQUENCY_GHZ;
    State_Machine_States[STATE_ACTIVATE_RX_CFG].txPrn                                           = DEFAULT_PRN;
    State_Machine_States[STATE_ACTIVATE_RX_CFG].rxPrn                                           = DEFAULT_PRN;
    State_Machine_States[STATE_ACTIVATE_RX_CFG].goldCodeNumBits                                 = GOLD_CODE_NUM_BITS;
    State_Machine_States[STATE_ACTIVATE_RX_CFG].stateTransitionTable[SM_MSG_INIT]               = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_ACTIVATE_RX_CFG].stateTransitionTable[SM_MSG_STANDBY]            = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_ACTIVATE_RX_CFG].stateTransitionTable[SM_MSG_RX_START_CODE]      = &State_Machine_States[STATE_RX_START_CODE];
    State_Machine_States[STATE_ACTIVATE_RX_CFG].stateTransitionTable[SM_MSG_RX_RESPONSE_CODE]   = &State_Machine_States[STATE_RX_RESPONSE_CODE];
    State_Machine_States[STATE_ACTIVATE_RX_CFG].stateTransitionTable[SM_MSG_FAILED]             = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_ACTIVATE_RX_CFG].stateTransitionTable[SM_MSG_CANCELLED]          = &State_Machine_States[STATE_CANCELLED];

    // RX Start Code
    State_Machine_States[STATE_RX_START_CODE].stateMachine                                      = &State_Machine;
    State_Machine_States[STATE_RX_START_CODE].stateExecutionFunction                            = SM_Func_Rx;
    State_Machine_States[STATE_RX_START_CODE].stateNumber                                       = STATE_RX_START_CODE;
    State_Machine_States[STATE_RX_START_CODE].rxFrequencyInGhz                                  = RX_FREQUENCY_GHZ;
    State_Machine_States[STATE_RX_START_CODE].txPrn                                             = DEFAULT_PRN;
    State_Machine_States[STATE_RX_START_CODE].rxPrn                                             = DEFAULT_PRN;
    State_Machine_States[STATE_RX_START_CODE].goldCodeNumBits                                   = GOLD_CODE_NUM_BITS;
    State_Machine_States[STATE_RX_START_CODE].stateTransitionTable[SM_MSG_INIT]                 = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_RX_START_CODE].stateTransitionTable[SM_MSG_STANDBY]              = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_RX_START_CODE].stateTransitionTable[SM_MSG_SENSOR_STARTED]       = &State_Machine_States[STATE_RX_CONTINUE];
    State_Machine_States[STATE_RX_START_CODE].stateTransitionTable[SM_MSG_DSS_REPORTS_FAILURE]  = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_RX_START_CODE].stateTransitionTable[SM_MSG_FAILED]               = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_RX_START_CODE].stateTransitionTable[SM_MSG_CANCELLED]            = &State_Machine_States[STATE_CANCELLED];

    // RX Response Code
    State_Machine_States[STATE_RX_RESPONSE_CODE].stateMachine                                   = &State_Machine;
    State_Machine_States[STATE_RX_RESPONSE_CODE].stateExecutionFunction                         = SM_Func_Rx;
    State_Machine_States[STATE_RX_RESPONSE_CODE].stateNumber                                    = STATE_RX_RESPONSE_CODE;
    State_Machine_States[STATE_RX_RESPONSE_CODE].rxFrequencyInGhz                               = RX_FREQUENCY_GHZ;
    State_Machine_States[STATE_RX_RESPONSE_CODE].txPrn                                          = DEFAULT_PRN;
    State_Machine_States[STATE_RX_RESPONSE_CODE].rxPrn                                          = DEFAULT_PRN;
    State_Machine_States[STATE_RX_RESPONSE_CODE].goldCodeNumBits                                = GOLD_CODE_NUM_BITS;
    State_Machine_States[STATE_RX_RESPONSE_CODE].stateTransitionTable[SM_MSG_INIT]              = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_RX_RESPONSE_CODE].stateTransitionTable[SM_MSG_STANDBY]           = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_RX_RESPONSE_CODE].stateTransitionTable[SM_MSG_COMPLETED]         = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_RX_RESPONSE_CODE].stateTransitionTable[SM_MSG_FAILED]            = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_RX_RESPONSE_CODE].stateTransitionTable[SM_MSG_CANCELLED]         = &State_Machine_States[STATE_CANCELLED];
    State_Machine_States[STATE_RX_RESPONSE_CODE].stateTransitionTable[SM_MSG_NO_CODE_DETECT]    = &State_Machine_States[STATE_ACTIVATE_TX_CFG];

    // RX Continue
    State_Machine_States[STATE_RX_CONTINUE].stateMachine                                = &State_Machine;
    State_Machine_States[STATE_RX_CONTINUE].stateExecutionFunction                      = SM_Func_No_Operation;
    State_Machine_States[STATE_RX_CONTINUE].stateNumber                                 = STATE_RX_CONTINUE;
    State_Machine_States[STATE_RX_CONTINUE].rxFrequencyInGhz                            = RX_FREQUENCY_GHZ;
    State_Machine_States[STATE_RX_CONTINUE].txPrn                                       = DEFAULT_PRN;
    State_Machine_States[STATE_RX_CONTINUE].rxPrn                                       = DEFAULT_PRN;
    State_Machine_States[STATE_RX_CONTINUE].goldCodeNumBits                             = GOLD_CODE_NUM_BITS;
    State_Machine_States[STATE_RX_CONTINUE].stateTransitionTable[SM_MSG_INIT]           = &State_Machine_States[STATE_INIT];
    State_Machine_States[STATE_RX_CONTINUE].stateTransitionTable[SM_MSG_STANDBY]        = &State_Machine_States[STATE_STANDBY];
    State_Machine_States[STATE_RX_CONTINUE].stateTransitionTable[SM_MSG_CODE_DETECT]    = &State_Machine_States[STATE_RX_CONTINUE];
    State_Machine_States[STATE_RX_CONTINUE].stateTransitionTable[SM_MSG_NO_CODE_DETECT] = &State_Machine_States[STATE_RX_CONTINUE];
    State_Machine_States[STATE_RX_CONTINUE].stateTransitionTable[SM_MSG_FAILED]         = &State_Machine_States[STATE_FAILED];
    State_Machine_States[STATE_RX_CONTINUE].stateTransitionTable[SM_MSG_CANCELLED]      = &State_Machine_States[STATE_CANCELLED];

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
