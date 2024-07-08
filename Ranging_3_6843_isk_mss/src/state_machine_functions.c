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
#include <inc/ranging_dpc_interface.h>
#include <inc/uart_logging.h>
#include <inc/ranging_mss.h>                // For Ranging_debugAssert
#include <shared/ranging_mailbox.h>         // For interprocess communications with the DSP core
#include <stdio.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  DEFINES
 * ----------------------------------------------------------------------------------------------------------------- */

#define ENABLE_UART_LOGGING
#undef ENABLE_UART_LOGGING

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Global Variables
 * ----------------------------------------------------------------------------------------------------------------- */

extern const char *  message_to_string_table[SM_MSG_TOTAL_COUNT];
extern const char *  state_to_string_table[STATE_TOTAL_COUNT];
extern const char* slotTypeNames[NUMBER_OF_SLOT_TYPES];
extern StateMachine_t State_Machine;

// From ranging_cli.c
extern int32_t Ranging_SetBaseConfiguration( );
extern int32_t Ranging_CreateReceiveConfiguration(float frequencyInGhz);
extern int32_t Ranging_CreateTransmitConfiguration(float frequencyInGhz, uint8_t numGoldCodeBits, uint16_t goldCodePrn);
extern int32_t Ranging_ActivateReceiveConfiguration( );
extern int32_t Ranging_ActivateTransmitConfiguration( );

// From mss_main.c
extern int32_t Ranging_dataPathConfig (uint16_t rxPrn);
extern void Ranging_dataPathStart (void);
extern Ranging_MSS_MCB    gMmwMssMCB;

// From mss_ipc_mailboc_task.c
extern timeLowHighRegisters_t  g_DSSTime;

int32_t MMWave_config_internal (MMWave_Handle mmWaveHandle, MMWave_CtrlCfg* ptrControlCfg, int32_t* errCode);

int32_t Ranging_mmWaveCtrlStop (void);

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Local Variables
 * ----------------------------------------------------------------------------------------------------------------- */

char errorString[256];

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Functions
 * ----------------------------------------------------------------------------------------------------------------- */

void synchronizeRadio(State_Information_Ptr_t p_stateInfo)
{
    if(Task_self() == p_stateInfo->stateMachine->taskHandle)
    {
        gMmwMssMCB.synchronized             = TRUE;
        Log_To_Uart( p_stateInfo->stateMachine->uartHandle, "Synchronized!\r\n" );
    }
}

// Define generic behavior that occurs whenever we receive an invalid event
void Service_Null_Message(State_Information_Ptr_t p_stateInfo, uint16_t message_id )
{
    StateMachine_Ptr_t sm = p_stateInfo->stateMachine;
    Log_To_Uart(    sm->uartHandle,
                    "NULL msg %u %s received for state %u %s\r\n",
                    message_id,
                    sm->message_to_string[message_id],
                    sm->currentState->stateNumber,
                    sm->state_to_string[sm->currentState->stateNumber] );
}

// Helper function to put the error into a standard string format
void Format_Error_String(State_Information_Ptr_t p_stateInfo,  char * error )
{
    rangingTimeSlot_Ptr_t p_currentTimeSlot;
    rangingTimeSlot_Ptr_t p_nextTimeSlot;

    p_currentTimeSlot   = getCurrentTimeSlot(&gMmwMssMCB.timeSlotList);
    p_nextTimeSlot      = getNextTimeSlot(&gMmwMssMCB.timeSlotList);
    snprintf(errorString, sizeof(errorString),
             "Error:%s in state %u %s, times entered: %u, Current TSlot: %s, Next TSlot: %s\r\n",
             error,
             p_stateInfo->stateNumber,
             state_to_string_table[p_stateInfo->stateNumber],
             p_stateInfo->timesEntered,
             slotTypeNames[p_currentTimeSlot->slotType],
             slotTypeNames[p_nextTimeSlot->slotType]);
}

void Send_State_Machine_Message( uint16_t message_id )
{
    MsgObj msg;
    msg.id = message_id;

    if(Task_self() == State_Machine.taskHandle)
    {
        Log_To_Uart(State_Machine.uartHandle,
                    "\t\t%s\tsent\t%s\r\n",
                    state_to_string_table[State_Machine.currentState->stateNumber],
                    message_to_string_table[message_id]);
    }


    if (!Mailbox_post(State_Machine.mbxHandle, &msg, BIOS_NO_WAIT))
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

void Leaving_State( State_Information_Ptr_t p_stateInfo )
{
    const char *state_string = p_stateInfo->stateMachine->state_to_string[p_stateInfo->stateNumber];
    if(Task_self() == p_stateInfo->stateMachine->taskHandle)
    {
        Log_To_Uart(p_stateInfo->stateMachine->uartHandle,
                    "\t\t%s\tEnd\r\n",
                    state_string);
    }
    p_stateInfo->stateEndTimeCycles = Cycleprofiler_getTimeStamp();
    p_stateInfo->stateDurationCycles = p_stateInfo->stateEndTimeCycles - p_stateInfo->stateStartTimeCycles;
}

void Entering_State( State_Information_Ptr_t p_stateInfo )
{
    const char *state_string = p_stateInfo->stateMachine->state_to_string[p_stateInfo->stateNumber];
    p_stateInfo->timesEntered++;
    p_stateInfo->stateStartTimeCycles = Cycleprofiler_getTimeStamp();
    if(Task_self() == p_stateInfo->stateMachine->taskHandle)
    {
        Log_To_Uart(p_stateInfo->stateMachine->uartHandle,
                  "\t\t%s\tStart\t%u\r\n",
                  state_string,
                  p_stateInfo->timesEntered);
    }
}

void SM_Func_Initialization( State_Information_Ptr_t p_stateInfo )
{
    //////////////////////////////////////////////////////////////////
    // CONFIGURE SENSOR
    if ( Ranging_SetBaseConfiguration( ) )
    {
        Format_Error_String(p_stateInfo, "Ranging_SetBaseConfiguration");
        Send_State_Machine_Message( SM_MSG_FAILED );
        return;
    }

    //////////////////////////////////////////////////////////////////
    // OPEN SENSOR
    if (gMmwMssMCB.sensorState == Ranging_SensorState_INIT)
    {
        if(Ranging_openSensor(true))
        {
            Format_Error_String(p_stateInfo, "Ranging_openSensor");
            Send_State_Machine_Message( SM_MSG_FAILED );
            return;
        }
        gMmwMssMCB.sensorState = Ranging_SensorState_OPENED;
    }

    Send_State_Machine_Message( SM_MSG_COMPLETED );
}

void SM_Func_Standby( State_Information_Ptr_t p_stateInfo )
{
    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        Ranging_stopSensor();
        gMmwMssMCB.sensorState = Ranging_SensorState_STOPPED;
    }
    gMmwMssMCB.synchronized = FALSE;
}

void SM_Func_Stop( State_Information_Ptr_t p_stateInfo )
{
    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        Ranging_stopSensor();
    }
}

void SM_Func_Update_List_Of_Timeslots( State_Information_Ptr_t p_stateInfo )
{
    // We set the timeslot to the last one in the circular linked list
    // This way, when we configure the next timeslot, it starts at the first timeslot
    rangingTimeSlot_Ptr_t p_currentTimeSlot;
    rangingTimeSlot_Ptr_t p_nextTimeSlot;
    rangingTimeSlot_Ptr_t lastTimeSlot;

    p_currentTimeSlot   = getCurrentTimeSlot(&gMmwMssMCB.timeSlotList);
    p_nextTimeSlot      = getNextTimeSlot(&gMmwMssMCB.timeSlotList);
    lastTimeSlot        = getTimeSlotAtIndex(&gMmwMssMCB.timeSlotList, gMmwMssMCB.timeSlotList.size - 1);

    while(p_currentTimeSlot != lastTimeSlot)
    {
        incrementCurrentTimeSlot(&gMmwMssMCB.timeSlotList);
        p_currentTimeSlot = getCurrentTimeSlot(&gMmwMssMCB.timeSlotList);
    }

    p_currentTimeSlot   = getCurrentTimeSlot(&gMmwMssMCB.timeSlotList);
    p_nextTimeSlot      = getNextTimeSlot(&gMmwMssMCB.timeSlotList);
    Log_To_Uart(    p_stateInfo->stateMachine->uartHandle,
                    "Cslot: %s \r\n",
                    slotTypeNames[p_currentTimeSlot->slotType] );
    Log_To_Uart(    p_stateInfo->stateMachine->uartHandle,
                    "Nslot: %s \r\n",
                    slotTypeNames[p_nextTimeSlot->slotType] );

    Send_State_Machine_Message( SM_MSG_COMPLETED );
}

void SM_Func_Cfg( State_Information_Ptr_t p_stateInfo )
{
    rangingTimeSlot_Ptr_t p_currentTimeSlot = getCurrentTimeSlot(&gMmwMssMCB.timeSlotList);
    rangingTimeSlot_Ptr_t p_nextTimeSlot = getNextTimeSlot(&gMmwMssMCB.timeSlotList);

    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        Format_Error_String(p_stateInfo, "Sensor already started SM_Func_Cfg");
        Send_State_Machine_Message( SM_MSG_FAILED );
        return;
    }

    // Configure the next time slot
    switch(p_nextTimeSlot->slotType)
    {
        case SLOT_TYPE_SYNCHRONIZATION_TX:
        case SLOT_TYPE_RANGING_START_CODE_TX:
        case SLOT_TYPE_RANGING_RESPONSE_CODE_TX:
            if ( Ranging_CreateTransmitConfiguration(p_nextTimeSlot->frequencyInGHz, p_nextTimeSlot->goldCodeNumBits, p_nextTimeSlot->prn ) )
            {
                Format_Error_String(p_stateInfo, "Ranging_CreateTransmitConfiguration");
                Send_State_Machine_Message( SM_MSG_FAILED );
                return;
            }
            break;

        case SLOT_TYPE_SYNCHRONIZATION_RX:
        case SLOT_TYPE_RANGING_START_CODE_RX:
        case SLOT_TYPE_RANGING_RESPONSE_CODE_RX:
            if ( Ranging_CreateReceiveConfiguration( p_nextTimeSlot->frequencyInGHz ) )
            {
                Format_Error_String(p_stateInfo, "Ranging_CreateReceiveConfiguration");
                Send_State_Machine_Message( SM_MSG_FAILED );
                return;
            }
            break;

        case SLOT_TYPE_NO_OP:
            // No configuration required
            break;
        default:
            Format_Error_String(p_stateInfo, "Bad time slot type for CFG\n");
            Send_State_Machine_Message( SM_MSG_FAILED );
            break;
    }

    // When the DSS receives this, it will compute the next gold code (if necessary)
    setNextTimeSlotOnDss(p_nextTimeSlot);

    Send_State_Machine_Message( SM_MSG_COMPLETED );
}

void SM_Func_Activate_Cfg( State_Information_Ptr_t p_stateInfo )
{
    rangingTimeSlot_Ptr_t p_nextTimeSlot = getNextTimeSlot(&gMmwMssMCB.timeSlotList);

    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        Format_Error_String(p_stateInfo, "Sensor already started SM_Func_Activate_Cfg");
        Send_State_Machine_Message( SM_MSG_FAILED );
        return;
    }

    switch(p_nextTimeSlot->slotType)
    {
        case SLOT_TYPE_SYNCHRONIZATION_TX:
        case SLOT_TYPE_RANGING_START_CODE_TX:
        case SLOT_TYPE_RANGING_RESPONSE_CODE_TX:
            if ( Ranging_ActivateTransmitConfiguration( ) )
            {
                Format_Error_String(p_stateInfo, "Ranging_ActivateTransmitConfiguration");
                Send_State_Machine_Message( SM_MSG_FAILED );
            }
            break;

        case SLOT_TYPE_SYNCHRONIZATION_RX:
        case SLOT_TYPE_RANGING_START_CODE_RX:
        case SLOT_TYPE_RANGING_RESPONSE_CODE_RX:
            if ( Ranging_ActivateReceiveConfiguration( ) )
            {
                Format_Error_String(p_stateInfo, "Ranging_ActivateReceiveConfiguration");
                Send_State_Machine_Message( SM_MSG_FAILED );
            }
            break;

        case SLOT_TYPE_NO_OP:
            // No configuration required
            break;

        default:
            Format_Error_String(p_stateInfo, "Bad time slot type for CFG\n");
            Send_State_Machine_Message( SM_MSG_FAILED );
            break;
    }

    Send_State_Machine_Message( SM_MSG_COMPLETED );
}

void SM_Func_Start_Execute( State_Information_Ptr_t p_stateInfo )
{
    rangingTimeSlot_Ptr_t p_nextTimeSlot = getNextTimeSlot(&gMmwMssMCB.timeSlotList);
    int32_t     errCode = 0;
    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        Format_Error_String(p_stateInfo, "Sensor already started SM_Func_Start_Sensor");
        Send_State_Machine_Message( SM_MSG_FAILED );
        return;
    }

    if (gMmwMssMCB.sensorState == Ranging_SensorState_INIT)
    {
        Format_Error_String(p_stateInfo, "Sensor not opened SM_Func_Start_Sensor");
        Send_State_Machine_Message( SM_MSG_FAILED );
        return;
    }

    switch(p_nextTimeSlot->slotType)
    {
        case SLOT_TYPE_SYNCHRONIZATION_TX:
        case SLOT_TYPE_RANGING_START_CODE_TX:
        case SLOT_TYPE_RANGING_RESPONSE_CODE_TX:
        case SLOT_TYPE_SYNCHRONIZATION_RX:
        case SLOT_TYPE_RANGING_START_CODE_RX:
        case SLOT_TYPE_RANGING_RESPONSE_CODE_RX:
            // 1. Configure the sensor through mmwavelink
            // 2. Configure the datapath running on the DSP core
            // 3. Start the sensor

            if(gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
            {
                gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames =
                        gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.numOfSubFrames;
            }
            else
            {
                gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames = 1;
            }

            //////////////////////////////////////////////////////////////////
            // 1. CONFIGURE SENSOR
            //Task_sleep(10);
            if (MMWave_config_internal (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.ctrlCfg, &errCode) < 0)
            {
                MMWave_ErrorLevel   errorLevel;
                int16_t             mmWaveErrorCode;
                int16_t             subsysErrorCode;

                /* Error: Report the error */
                MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
                System_printf ("Error: mmWave Config failed [Error code: %d Subsystem: %d]\n",
                                mmWaveErrorCode, subsysErrorCode);
                Format_Error_String(p_stateInfo, "Error MMWave_config");
                Send_State_Machine_Message( SM_MSG_FAILED );
                return;
            }

            //////////////////////////////////////////////////////////////////
            // 2. CONFIGURE DATAPATH
            // This is where we set the PRN for the DSS to look for
            if(Ranging_dataPathConfig(p_nextTimeSlot->prn))
            {
                Format_Error_String(p_stateInfo, "Error Ranging_dataPathConfig");
                Send_State_Machine_Message( SM_MSG_FAILED );
                return;
            }

            //////////////////////////////////////////////////////////////////
            // 3. START SENSOR
            //Ranging_dataPathStart();    // Starts the LVDS module

            // Send a command to the DSS to start the sensor
            // When the sensor is started, it triggers the MSS sensor start callback function
            // The callback function will call Send_Sensor_Started_Message( )
            // Which sends SM_MSG_SENSOR_STARTED to the state machine

            if(!gMmwMssMCB.synchronized)
            {
                // If we're not sync'd, but we're transmitting the sync message
                // then we are timing master - so we're sync'd
                if(p_nextTimeSlot->slotType == SLOT_TYPE_SYNCHRONIZATION_TX)
                {
                    synchronizeRadio(p_stateInfo);
                }

                // Once SYNC_IN works, we will configure the sensor to start upon SYNC_IN
                // The MSS will configure it, and the DSS will send SYNC_IN
                // Until then, we start it from the MSS
                // Ranging_startSensor();

                // Until SYNC_IN works, this just has the DSS call us back with a
                // DSS_REPORTS_NEXT_TIMESLOT_STARTED message
                cmdDssToStartSensorNow();
            }
            else
            {
                // Until SYNC_IN works, this just has the DSS call us back with a
                // DSS_REPORTS_NEXT_TIMESLOT_STARTED message
                // At the start of the next timeslot
                cmdDssToStartSensorAtNextTimeslot();
            }

            gMmwMssMCB.sensorState = Ranging_SensorState_STARTED;
            gMmwMssMCB.sensorStartCount++;
            break;

        case SLOT_TYPE_NO_OP:
            // Tell the DSS to wake us up at the next timeslot
            cmdDssToMsgMssAtNextTimeslot();
            break;

        default:
            Format_Error_String(p_stateInfo, "Bad time slot type for EXECUTE\n");
            Send_State_Machine_Message( SM_MSG_FAILED );
            break;
    }
}

void SM_Func_Executing( State_Information_Ptr_t p_stateInfo )
{
    // This function is triggered by a TIMESLOT_STARTED message
    rangingTimeSlot_Ptr_t p_currentTimeSlot;
    rangingTimeSlot_Ptr_t p_nextTimeSlot = getNextTimeSlot(&gMmwMssMCB.timeSlotList);

    // Once SYNC_IN works, this will be called from SM_Func_Execute_Cfg
    if(p_nextTimeSlot->slotType != SLOT_TYPE_NO_OP)
    {
        Ranging_startSensor();
    }

    // If we have never sync'd stay in this time slot (presumably RX SYNC, SLOT 0)
    if(gMmwMssMCB.synchronized)
    {
        incrementCurrentTimeSlot(&gMmwMssMCB.timeSlotList);
        p_nextTimeSlot->timesEntered++;
    }
    else
    {
        p_nextTimeSlot    = getNextTimeSlot(&gMmwMssMCB.timeSlotList);
        if(p_nextTimeSlot->slotType == SLOT_TYPE_SYNCHRONIZATION_RX)
        {
            incrementCurrentTimeSlot(&gMmwMssMCB.timeSlotList);
            p_nextTimeSlot->timesEntered++;
        }
    }

    p_currentTimeSlot = getCurrentTimeSlot(&gMmwMssMCB.timeSlotList);
    p_nextTimeSlot    = getNextTimeSlot(&gMmwMssMCB.timeSlotList);
    computeNextStartTime(p_currentTimeSlot, p_nextTimeSlot);

    Log_To_Uart(    p_stateInfo->stateMachine->uartHandle,
                    "Cslot: %s %u\r\n",
                    slotTypeNames[p_currentTimeSlot->slotType],
                    p_currentTimeSlot->timesEntered);
    Log_To_Uart(    p_stateInfo->stateMachine->uartHandle,
                    "Nslot: %s %u \r\n",
                    slotTypeNames[p_nextTimeSlot->slotType],
                    p_nextTimeSlot->timesEntered );

    // If this time slot is a no-op, go ahead and get ready for the next time slot
    // Otherwise, wait for a result from the DSS
    if(p_currentTimeSlot->slotType == SLOT_TYPE_NO_OP)
    {
        Send_State_Machine_Message( SM_MSG_CFG_NEXT_TIMESLOT );
    }
}

void SM_Func_Process_Result( State_Information_Ptr_t p_stateInfo )
{
    rangingTimeSlot_Ptr_t p_currentTimeSlot = getCurrentTimeSlot(&gMmwMssMCB.timeSlotList);
    rangingTimeSlot_Ptr_t p_nextTimeSlot    = getNextTimeSlot(&gMmwMssMCB.timeSlotList);

    // Stop the sensor
    Ranging_stopSensor();
    Log_To_Uart( p_stateInfo->stateMachine->uartHandle, "Stopped sensor\r\n" );

    switch(p_currentTimeSlot->slotType)
    {
        case SLOT_TYPE_SYNCHRONIZATION_TX:
            if(p_currentTimeSlot->slotStart.timeLow == 0 && p_currentTimeSlot->slotStart.timeHigh == 0)
            {
                // This is the first TX.
                // We need to set its start time, and update the next timeslot's start time.
                computeFirstStartTime(p_currentTimeSlot, &gMmwMssMCB.rangingData);
                computeNextStartTime(p_currentTimeSlot, p_nextTimeSlot);
            }
            break;

        case SLOT_TYPE_RANGING_START_CODE_TX:
        case SLOT_TYPE_RANGING_RESPONSE_CODE_TX:
            break;

        case SLOT_TYPE_SYNCHRONIZATION_RX:
            if(gMmwMssMCB.rangingData.detectionStats.wasCodeDetected)
            {
                // Is this the first successful sync?
                if(!gMmwMssMCB.synchronized)
                {
                    // Set the current timeslot's start time
                    computeFirstStartTime(p_currentTimeSlot, &gMmwMssMCB.rangingData);

                    // Set the next timeslot's start time
                    computeNextStartTime(p_currentTimeSlot, p_nextTimeSlot);
                }

                synchronizeRadio(p_stateInfo);
            }
            else if (!gMmwMssMCB.synchronized)
            {
                // No code was detected, and we have never been sync'd
                // Keep looking until we find a sync code
                Send_State_Machine_Message( SM_MSG_START_EXECUTING );
            }
            break;

        case SLOT_TYPE_RANGING_START_CODE_RX:
            if(gMmwMssMCB.rangingData.detectionStats.wasCodeDetected)
            {
                // Setup reply
                // we respond after RESPONSE_CODE_DELAY_DSP_CYCLES
                //
            }
            break;

        case SLOT_TYPE_RANGING_RESPONSE_CODE_RX:
            if(gMmwMssMCB.rangingData.detectionStats.wasCodeDetected)
            {
                // Compute range
            }
            break;

        default:
            Format_Error_String(p_stateInfo, "Bad time slot type for process result\n");
            Send_State_Machine_Message( SM_MSG_FAILED );
            break;
    }

    if(gMmwMssMCB.synchronized)
    {
        Send_State_Machine_Message( SM_MSG_CFG_NEXT_TIMESLOT );
    }
}

void SM_Func_Task_Completed_Successfully( State_Information_Ptr_t p_stateInfo )
{
    System_printf("Task Complete\n");
}

void SM_Func_Task_Failed( State_Information_Ptr_t p_stateInfo )
{
    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        Ranging_stopSensor();
    }

    System_printf("**********FAILURE\n");
    System_printf(errorString);
    System_printf("**********FAILURE\n");
    Send_State_Machine_Message( SM_MSG_COMPLETED );
}

void SM_Func_Task_Cancelled( State_Information_Ptr_t p_stateInfo )
{
    System_printf("**********Cancelled\n");
}
