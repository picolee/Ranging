/*
 * test_state_machines.c
 *
 *  Created on: Jun 16, 2024
 *      Author: LeeLemay
 */

/*
 * See state_machine.c for an overview of the state machine
 *
 * To make this code run:
 * uncomment the initialization function in mss_main.c
 * and //Send_Test_SM_New_Timeslot_Started_Message(); in mss_ipc_mailbox>task.c
 * *
 * This runs two timing tests:
 * time to transition between two states
 * time elapsed when the DSS is set to ping us back after 1000 microseconds
 *
 * Here are the results from running the code: (in 200 MHz clock cycles)
 * State Transition Test:
        689183868 Low:  947
        689230959 Average:      960
        689286901 High:         1882
   DSS 1000 us callback test:
        713744995 Low:  198791
        713798795 Average:      200095
        713861338 High:         318423
 */

#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <inc/state_machine.h>
#include <inc/state_machine_definitions.h>
#include <inc/state_machine_functions.h>
#include <inc/ranging_mss.h>                // For Ranging_debugAssert
#include <inc/timeslot_list.h>
#include <inc/uart_logging.h>
#include <inc/state_machine_definitions.h>
#include <inc/state_machine_functions.h>
#include <shared/ranging_rfConfig.h>
#include <shared/ranging_mailbox.h>         // For interprocess communications with the DSP core

#define DSP_CYCLES_PER_US 600

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Typedefs
 * ----------------------------------------------------------------------------------------------------------------- */

/* Active Task Type */
// Keep in synch with robot_task_strings defined in Robot.c
typedef enum
{
    TEST_SM_INIT              = 0U,
    TEST_SM_STATE_TRANSITION_TIMING_TEST_ONE,
    TEST_SM_STATE_TRANSITION_TIMING_TEST_TWO,
    TEST_SM_START_DSS_SYNC,
    TEST_SM_END_DSS_SYNC,
    TEST_SM_DSS_COMMS_TIMING_TEST_ONE,
    TEST_SM_DSS_COMMS_TIMING_TEST_TWO,
    TEST_SM_REPORT_DSS_TIMING_LOOP,
    TEST_SM_REPORT_STATE_TRANS_TIMING_LOOP,
    TEST_SM_DISP_STATE_TRANS_TO_UART,
    TEST_SM_DISP_SYNC_TO_UART,
    TEST_SM_DISP_DSS_COMMS_TO_UART,
    TEST_SM_TOTAL_COUNT
} Test_DSS_Timer_Callback_State_Enum_t;

typedef enum
{
    TEST_SM_MSG_INIT       = 0U,
    TEST_SM_MSG_DSS_ACK_RECEIVED,
    TEST_SM_MSG_STATE_COMPLETED,
    TEST_SM_MSG_TIME_RECEIVED,
    TEST_SM_MSG_LOOP,
    TEST_SM_MSG_TOTAL_COUNT
} Test_DSS_Timer_Callback_State_Messages_t;

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Global Variables
 * ----------------------------------------------------------------------------------------------------------------- */

const char *  test_dss_message_to_string_table[TEST_SM_MSG_TOTAL_COUNT] = {0};
const char *  test_dss_state_to_string_table[TEST_SM_TOTAL_COUNT] = {0};

StateMachine_t Test_SM;


// This array holds the State_Information_t that define the behavior of each state
State_Information_t Test_SM_States[TEST_SM_TOTAL_COUNT]     = {0};

/*! @brief      Schedule for the radio */
circularLinkedTimeSlotList_t    testTimeSlotList;

extern timeLowHighRegisters_t  g_DSSTime;

#define NUM_TIMING_TESTS 100
uint32_t g_timingTestStart;
uint32_t g_timingTestEnd;
uint32_t g_timingTestCount = 0;
uint32_t g_timingTestResults[NUM_TIMING_TESTS];


/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                   Local Variable Declarations
 * ----------------------------------------------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                   Local Function Declarations
 * ----------------------------------------------------------------------------------------------------------------- */

void State_Machine_Entry_Point(StateMachine_Ptr_t p_stateMachine,
                               State_Information_Ptr_t p_state_array,
                               uint16_t totalStates,
                               uint8_t taskPriority,
                               UART_Handle uart);

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                   External Functions
 * ----------------------------------------------------------------------------------------------------------------- */

//----------------------------------------------------------------------------------------------------------------- *
//                                                   Local Functions
//----------------------------------------------------------------------------------------------------------------- *

///////////////////////////////////////////////////////////////////////////////////////////
//  MESSAGES
///////////////////////////////////////////////////////////////////////////////////////////

void Send_Test_SM_Message( uint16_t message_id )
{
    MsgObj msg;
    msg.id = message_id;

    // Calling the UART from some tasks causes an error
    // Only call it from the state machine task
    if(Task_self() == Test_SM.taskHandle)
    {
        Log_To_Uart(Test_SM.uartHandle,
                    "\t\t%s\tsent\t%s\r\n",
                    test_dss_state_to_string_table[Test_SM.currentState->stateNumber],
                    test_dss_message_to_string_table[message_id]);
    }


    if (!Mailbox_post(Test_SM.mbxHandle, &msg, BIOS_NO_WAIT))
    {
        // Make sure we're not in an ISR
        if(BIOS_getThreadType() == BIOS_ThreadType_Task)
        {
          System_printf("Mailbox Write Failed: ID = %d.\n", msg.id);
        }
        else
        {
          Ranging_debugAssert (0);
        }
    }
}

void Send_Test_SM_New_Timeslot_Started_Message()
{
    Send_Test_SM_Message( TEST_SM_MSG_DSS_ACK_RECEIVED );
}

void Send_Test_SM_State_Completed_Message()
{
    Send_Test_SM_Message( TEST_SM_MSG_STATE_COMPLETED );
}

void Send_Test_SM_ACK_Message()
{
    Send_Test_SM_Message( TEST_SM_MSG_DSS_ACK_RECEIVED );
}

void Send_Test_SM_Init_Message( )
{
    Send_Test_SM_Message( TEST_SM_MSG_INIT );
}

void Send_Test_SM_Loop_Message( )
{
    Send_Test_SM_Message( TEST_SM_MSG_LOOP );
}

///////////////////////////////////////////////////////////////////////////////////////////
//  STATE FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////

void Test_SM_Func_Initialization( State_Information_Ptr_t p_stateInfo )
{
    // Setup two time slots
    rangingTimeSlot_t slot;
    initTimeSlotList(&testTimeSlotList);
    slot.slotType = SLOT_TYPE_NO_OP;
    slot.frequencyInGHz     = 63.95;
    slot.prn                = 3;
    slot.goldCodeNumBits    = 6;
    slot.slotStart.timeHigh = 0;
    slot.slotStart.timeLow  = 0;
    slot.slotDurationDSPCycles  = 1000*DSP_CYCLES_PER_US;
    initializeTimeSlot(
            &slot,
            slot.slotType,
            slot.slotStart.timeLow,
            slot.slotStart.timeHigh,
            slot.slotDurationDSPCycles,
            0,
            0,
            slot.frequencyInGHz,
            slot.prn,
            slot.goldCodeNumBits);
    addTimeSlotToEnd(&testTimeSlotList, slot);
    addTimeSlotToEnd(&testTimeSlotList, slot);

    Cycleprofiler_init();

    Task_sleep(10);

    Send_Test_SM_State_Completed_Message();
}

void Test_SM_Func_Timing_Test_One( State_Information_Ptr_t p_stateInfo )
{
    g_timingTestStart = Cycleprofiler_getTimeStamp();
    Send_Test_SM_State_Completed_Message();
}

void Test_SM_Func_Timing_Test_Two( State_Information_Ptr_t p_stateInfo )
{
    g_timingTestEnd = Cycleprofiler_getTimeStamp();
    g_timingTestResults[g_timingTestCount] = g_timingTestEnd - g_timingTestStart;
    g_timingTestCount++;
    if(Task_self() == p_stateInfo->stateMachine->taskHandle)
    {
        Log_To_Uart(    p_stateInfo->stateMachine->uartHandle,
                        "Test: %u\r\n",
                        g_timingTestCount );
    }
    if(g_timingTestCount >= NUM_TIMING_TESTS)
    {
        g_timingTestCount = 0;
        Send_Test_SM_State_Completed_Message();
    }
    else
    {
        Send_Test_SM_Loop_Message();
    }
}

void Test_SM_Func_Timing_Report( State_Information_Ptr_t p_stateInfo )
{
    uint32_t index;
    uint32_t average    = 0;
    uint32_t low        = UINT32_MAX;
    uint32_t high       = 0;

    Priority_Log_To_Uart(        p_stateInfo->stateMachine->uartHandle,
                                 "Timing results:\r\n");

    for(index = 0; index < NUM_TIMING_TESTS; index++)
    {
        Priority_Log_To_Uart(    p_stateInfo->stateMachine->uartHandle,
                                 "%u\r\n",
                                 g_timingTestResults[index] );
        average += g_timingTestResults[index];
        if(g_timingTestResults[index] < low)
        {
            low = g_timingTestResults[index];
        }
        if(g_timingTestResults[index] > high)
        {
            high = g_timingTestResults[index];
        }
    }
    average /= NUM_TIMING_TESTS;

    Priority_Log_To_Uart(    p_stateInfo->stateMachine->uartHandle,
                             "Low: \t%u\r\n",
                             low );
    Priority_Log_To_Uart(    p_stateInfo->stateMachine->uartHandle,
                             "Average: \t%u\r\n",
                             average );
    Priority_Log_To_Uart(    p_stateInfo->stateMachine->uartHandle,
                             "High: \t%u\r\n",
                             high );

    Send_Test_SM_State_Completed_Message();
}

void Test_SM_Func_Start_Sync( State_Information_Ptr_t p_stateInfo )
{
    // Get the time from the DSS
    ping();
}

void Test_SM_Func_End_Sync( State_Information_Ptr_t p_stateInfo )
{
    // Setup two time slots
    rangingTimeSlot_Ptr_t p_currentTimeSlot;
    rangingTimeSlot_Ptr_t p_nextTimeSlot;
    p_currentTimeSlot = getCurrentTimeSlot(&testTimeSlotList);
    p_nextTimeSlot    = getNextTimeSlot(&testTimeSlotList);

    // Use the time that we got from the DSS to initialize the timeslots
    p_currentTimeSlot->slotStart.timeLow    = g_DSSTime.timeLow;
    p_currentTimeSlot->slotStart.timeHigh   = g_DSSTime.timeHigh;
    computeNextStartTime(p_currentTimeSlot, p_nextTimeSlot);

    // Now we have the time
    Send_Test_SM_State_Completed_Message();
}

void Test_SM_Func_Start_Loop( State_Information_Ptr_t p_stateInfo )
{
    rangingTimeSlot_Ptr_t p_currentTimeSlot;
    rangingTimeSlot_Ptr_t p_nextTimeSlot;

    g_timingTestStart = Cycleprofiler_getTimeStamp();

    incrementCurrentTimeSlot(&testTimeSlotList);
    p_currentTimeSlot = getCurrentTimeSlot(&testTimeSlotList);
    p_nextTimeSlot    = getNextTimeSlot(&testTimeSlotList);
    computeNextStartTime(p_currentTimeSlot, p_nextTimeSlot);

    // 1. Set the target callback time
    setNextTimeSlotOnDss(p_nextTimeSlot);

    // 2. Have the DSS alert us at that time
    cmdDssToMsgMssAtNextTimeslot();
}

void Test_SM_Func_End_Loop( State_Information_Ptr_t p_stateInfo )
{
    // The DSS sent us the
    g_timingTestEnd = Cycleprofiler_getTimeStamp();
    g_timingTestResults[g_timingTestCount] = g_timingTestEnd - g_timingTestStart;
    g_timingTestCount++;
    if(Task_self() == p_stateInfo->stateMachine->taskHandle)
    {
        Log_To_Uart(    p_stateInfo->stateMachine->uartHandle,
                        "Test: %u\r\n",
                        g_timingTestCount );
    }
    if(g_timingTestCount >= NUM_TIMING_TESTS)
    {
        g_timingTestCount = 0;
        Send_Test_SM_State_Completed_Message();
    }
    else
    {
        Send_Test_SM_Loop_Message();
    }
}

void Test_SM_Func_Display_Previous_State_To_UART( State_Information_Ptr_t p_stateInfo )
{
    StateMachine_Ptr_t sm = p_stateInfo->stateMachine;

    Priority_Log_To_Uart(        sm->uartHandle,
                                 "Just left %s\r\n",
                                 sm->state_to_string[p_stateInfo->previousStateInfo_ptr->stateNumber]);

    Send_Test_SM_State_Completed_Message();
}

///////////////////////////////////////////////////////////////////////////////////////////
//  TASK THREAD
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
//  INITIALIZATION
///////////////////////////////////////////////////////////////////////////////////////////

void Define_Test_DSS_Timer_Callback_State_Machine( void )
{
    // Configure the string table
    uint16_t index;
    for(index = 0; index < TEST_SM_MSG_TOTAL_COUNT; index++)
    {
        test_dss_message_to_string_table[index] = "TEST_DSS_MSG_UNINIT";
    }
    test_dss_message_to_string_table[TEST_SM_MSG_INIT]              = "T_SM_MSG_INIT";
    test_dss_message_to_string_table[TEST_SM_MSG_DSS_ACK_RECEIVED]  = "T_SM_MSG_DSS_ACK";
    test_dss_message_to_string_table[TEST_SM_MSG_STATE_COMPLETED]   = "T_SM_MSG_STATE_COMP";
    test_dss_message_to_string_table[TEST_SM_MSG_TIME_RECEIVED]     = "T_SM_MSG_TIME_RX";
    test_dss_message_to_string_table[TEST_SM_MSG_LOOP]              = "TEST_SM_MSG_LOOP";
    Test_SM.message_to_string                        = test_dss_message_to_string_table;

    for(index = 0; index < TEST_SM_TOTAL_COUNT; index++)
    {
        test_dss_state_to_string_table[index] = "TEST_SM_UNINIT";
    }
    test_dss_state_to_string_table[TEST_SM_INIT]                                = "TEST_SM_INIT";
    test_dss_state_to_string_table[TEST_SM_START_DSS_SYNC]                      = "TEST_SM_SYNC1";
    test_dss_state_to_string_table[TEST_SM_END_DSS_SYNC]                        = "TEST_SM_SYNC2";
    test_dss_state_to_string_table[TEST_SM_DSS_COMMS_TIMING_TEST_ONE]           = "TEST_SM_DSS_COMMS_TEST1";
    test_dss_state_to_string_table[TEST_SM_DSS_COMMS_TIMING_TEST_TWO]           = "TEST_SM_DSS_COMMS_TEST2";
    test_dss_state_to_string_table[TEST_SM_STATE_TRANSITION_TIMING_TEST_ONE]    = "TEST_SM_STATE_TRANS_TEST1";
    test_dss_state_to_string_table[TEST_SM_STATE_TRANSITION_TIMING_TEST_TWO]    = "TEST_SM_STATE_TRANS_TEST2";
    test_dss_state_to_string_table[TEST_SM_REPORT_STATE_TRANS_TIMING_LOOP]      = "TEST_SM_STATE_TRANS_REPORT";
    test_dss_state_to_string_table[TEST_SM_REPORT_DSS_TIMING_LOOP]              = "TEST_SM_REPORT_DSS_TIMING_LOOP";
    test_dss_state_to_string_table[TEST_SM_DISP_SYNC_TO_UART]                   = "TEST_SM_DISP_SYNC_TO_UART";
    test_dss_state_to_string_table[TEST_SM_DISP_DSS_COMMS_TO_UART]              = "TEST_SM_DISP_DSS_COMMS_TO_UART";
    test_dss_state_to_string_table[TEST_SM_DISP_STATE_TRANS_TO_UART]            = "TEST_SM_DISP_STATE_TRANS_TO_UART";
    Test_SM.state_to_string          = test_dss_state_to_string_table;

    // Initialization
    Test_SM_States[TEST_SM_INIT].stateMachine                                       = &Test_SM;
    Test_SM_States[TEST_SM_INIT].stateExecutionFunction                             = Test_SM_Func_Initialization;
    Test_SM_States[TEST_SM_INIT].stateNumber                                        = TEST_SM_INIT;
    Test_SM_States[TEST_SM_INIT].previousStateInfo_ptr                              = &Test_SM_States[TEST_SM_INIT];
    Test_SM_States[TEST_SM_INIT].stateTransitionTable[TEST_SM_MSG_INIT]             = &Test_SM_States[STATE_INIT];
    Test_SM_States[TEST_SM_INIT].stateTransitionTable[TEST_SM_MSG_STATE_COMPLETED]  = &Test_SM_States[TEST_SM_STATE_TRANSITION_TIMING_TEST_ONE];

    // SM State Transition Timing Test
    Test_SM_States[TEST_SM_STATE_TRANSITION_TIMING_TEST_ONE].stateMachine                                       = &Test_SM;
    Test_SM_States[TEST_SM_STATE_TRANSITION_TIMING_TEST_ONE].stateExecutionFunction                             = Test_SM_Func_Timing_Test_One;
    Test_SM_States[TEST_SM_STATE_TRANSITION_TIMING_TEST_ONE].stateNumber                                        = TEST_SM_STATE_TRANSITION_TIMING_TEST_ONE;
    Test_SM_States[TEST_SM_STATE_TRANSITION_TIMING_TEST_ONE].stateTransitionTable[TEST_SM_MSG_STATE_COMPLETED]  = &Test_SM_States[TEST_SM_STATE_TRANSITION_TIMING_TEST_TWO];

    Test_SM_States[TEST_SM_STATE_TRANSITION_TIMING_TEST_TWO].stateMachine                                       = &Test_SM;
    Test_SM_States[TEST_SM_STATE_TRANSITION_TIMING_TEST_TWO].stateExecutionFunction                             = Test_SM_Func_Timing_Test_Two;
    Test_SM_States[TEST_SM_STATE_TRANSITION_TIMING_TEST_TWO].stateNumber                                        = TEST_SM_STATE_TRANSITION_TIMING_TEST_TWO;
    Test_SM_States[TEST_SM_STATE_TRANSITION_TIMING_TEST_TWO].stateTransitionTable[TEST_SM_MSG_LOOP]             = &Test_SM_States[TEST_SM_STATE_TRANSITION_TIMING_TEST_ONE];
    Test_SM_States[TEST_SM_STATE_TRANSITION_TIMING_TEST_TWO].stateTransitionTable[TEST_SM_MSG_STATE_COMPLETED]  = &Test_SM_States[TEST_SM_DISP_STATE_TRANS_TO_UART];

    Test_SM_States[TEST_SM_DISP_STATE_TRANS_TO_UART].stateMachine                                       = &Test_SM;
    Test_SM_States[TEST_SM_DISP_STATE_TRANS_TO_UART].stateExecutionFunction                             = Test_SM_Func_Display_Previous_State_To_UART;
    Test_SM_States[TEST_SM_DISP_STATE_TRANS_TO_UART].stateNumber                                        = TEST_SM_DISP_SYNC_TO_UART;
    Test_SM_States[TEST_SM_DISP_STATE_TRANS_TO_UART].stateTransitionTable[TEST_SM_MSG_STATE_COMPLETED]  = &Test_SM_States[TEST_SM_REPORT_STATE_TRANS_TIMING_LOOP];

    Test_SM_States[TEST_SM_REPORT_STATE_TRANS_TIMING_LOOP].stateMachine                                       = &Test_SM;
    Test_SM_States[TEST_SM_REPORT_STATE_TRANS_TIMING_LOOP].stateExecutionFunction                             = Test_SM_Func_Timing_Report;
    Test_SM_States[TEST_SM_REPORT_STATE_TRANS_TIMING_LOOP].stateNumber                                        = TEST_SM_REPORT_STATE_TRANS_TIMING_LOOP;
    Test_SM_States[TEST_SM_REPORT_STATE_TRANS_TIMING_LOOP].stateTransitionTable[TEST_SM_MSG_STATE_COMPLETED]  = &Test_SM_States[TEST_SM_START_DSS_SYNC];

    // Sync
    Test_SM_States[TEST_SM_START_DSS_SYNC].stateMachine                                         = &Test_SM;
    Test_SM_States[TEST_SM_START_DSS_SYNC].stateExecutionFunction                               = Test_SM_Func_Start_Sync;
    Test_SM_States[TEST_SM_START_DSS_SYNC].stateNumber                                          = TEST_SM_START_DSS_SYNC;
    Test_SM_States[TEST_SM_START_DSS_SYNC].stateTransitionTable[TEST_SM_MSG_DSS_ACK_RECEIVED]   = &Test_SM_States[TEST_SM_END_DSS_SYNC];

    Test_SM_States[TEST_SM_END_DSS_SYNC].stateMachine                                       = &Test_SM;
    Test_SM_States[TEST_SM_END_DSS_SYNC].stateExecutionFunction                             = Test_SM_Func_End_Sync;
    Test_SM_States[TEST_SM_END_DSS_SYNC].stateNumber                                        = TEST_SM_END_DSS_SYNC;
    Test_SM_States[TEST_SM_END_DSS_SYNC].stateTransitionTable[TEST_SM_MSG_STATE_COMPLETED]  = &Test_SM_States[TEST_SM_DISP_SYNC_TO_UART];

    Test_SM_States[TEST_SM_DISP_SYNC_TO_UART].stateMachine                                       = &Test_SM;
    Test_SM_States[TEST_SM_DISP_SYNC_TO_UART].stateExecutionFunction                             = Test_SM_Func_Display_Previous_State_To_UART;
    Test_SM_States[TEST_SM_DISP_SYNC_TO_UART].stateNumber                                        = TEST_SM_DISP_SYNC_TO_UART;
    Test_SM_States[TEST_SM_DISP_SYNC_TO_UART].stateTransitionTable[TEST_SM_MSG_STATE_COMPLETED]  = &Test_SM_States[TEST_SM_DSS_COMMS_TIMING_TEST_ONE];

    // Start Loop
    Test_SM_States[TEST_SM_DSS_COMMS_TIMING_TEST_ONE].stateMachine                                          = &Test_SM;
    Test_SM_States[TEST_SM_DSS_COMMS_TIMING_TEST_ONE].stateExecutionFunction                                = Test_SM_Func_Start_Loop;
    Test_SM_States[TEST_SM_DSS_COMMS_TIMING_TEST_ONE].stateNumber                                           = TEST_SM_DSS_COMMS_TIMING_TEST_ONE;
    Test_SM_States[TEST_SM_DSS_COMMS_TIMING_TEST_ONE].stateTransitionTable[TEST_SM_MSG_DSS_ACK_RECEIVED]    = &Test_SM_States[TEST_SM_DSS_COMMS_TIMING_TEST_TWO];

    // End Loop
    Test_SM_States[TEST_SM_DSS_COMMS_TIMING_TEST_TWO].stateMachine                                      = &Test_SM;
    Test_SM_States[TEST_SM_DSS_COMMS_TIMING_TEST_TWO].stateExecutionFunction                            = Test_SM_Func_End_Loop;
    Test_SM_States[TEST_SM_DSS_COMMS_TIMING_TEST_TWO].stateNumber                                       = TEST_SM_DSS_COMMS_TIMING_TEST_TWO;
    Test_SM_States[TEST_SM_DSS_COMMS_TIMING_TEST_TWO].stateTransitionTable[TEST_SM_MSG_STATE_COMPLETED] = &Test_SM_States[TEST_SM_DISP_DSS_COMMS_TO_UART];
    Test_SM_States[TEST_SM_DSS_COMMS_TIMING_TEST_TWO].stateTransitionTable[TEST_SM_MSG_LOOP]             = &Test_SM_States[TEST_SM_DSS_COMMS_TIMING_TEST_ONE];

    Test_SM_States[TEST_SM_DISP_DSS_COMMS_TO_UART].stateMachine                                         = &Test_SM;
    Test_SM_States[TEST_SM_DISP_DSS_COMMS_TO_UART].stateExecutionFunction                               = Test_SM_Func_Display_Previous_State_To_UART;
    Test_SM_States[TEST_SM_DISP_DSS_COMMS_TO_UART].stateNumber                                          = TEST_SM_DISP_SYNC_TO_UART;
    Test_SM_States[TEST_SM_DISP_DSS_COMMS_TO_UART].stateTransitionTable[TEST_SM_MSG_STATE_COMPLETED]    = &Test_SM_States[TEST_SM_REPORT_DSS_TIMING_LOOP];

    Test_SM_States[TEST_SM_REPORT_DSS_TIMING_LOOP].stateMachine                                         = &Test_SM;
    Test_SM_States[TEST_SM_REPORT_DSS_TIMING_LOOP].stateExecutionFunction                               = Test_SM_Func_Timing_Report;
    Test_SM_States[TEST_SM_REPORT_DSS_TIMING_LOOP].stateNumber                                          = TEST_SM_REPORT_DSS_TIMING_LOOP;
    Test_SM_States[TEST_SM_REPORT_DSS_TIMING_LOOP].stateTransitionTable[TEST_SM_MSG_STATE_COMPLETED]    = &Test_SM_States[TEST_SM_STATE_TRANSITION_TIMING_TEST_ONE];
}

void Test_DSS_Timer_Callback_State_Machine_Init(uint8_t taskPriority, UART_Handle uart)
{
    // Define all of the states and string tables for the main state machine
    Define_Test_DSS_Timer_Callback_State_Machine();

    // Initialize the state machine mailboxes and thread
    State_Machine_Entry_Point(&Test_SM,
                              &(Test_SM_States[0]),
                              TEST_SM_TOTAL_COUNT,
                              taskPriority, uart);

    Send_Test_SM_Init_Message( );
}
