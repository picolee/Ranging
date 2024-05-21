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
#include <inc/ranging_mss.h>                // For Ranging_debugAssert
#include <stdio.h>
#include <ti/utils/cli/cli.h>

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  DEFINES
 * ----------------------------------------------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Global Variables
 * ----------------------------------------------------------------------------------------------------------------- */

extern const char *  message_to_string_table[SM_MSG_TOTAL_COUNT];
extern const char *  state_to_string_table[STATE_TOTAL_COUNT];
extern StateMachine_t State_Machine;

// From ranging_cli.c
extern int32_t Ranging_SetBaseConfiguration( );
extern int32_t Ranging_CreateReceiveConfiguration(float frequencyInGhz);
extern int32_t Ranging_CreateTransmitConfiguration(float frequencyInGhz, uint8_t numGoldCodeBits, uint16_t goldCodePrn);
extern int32_t Ranging_ActivateReceiveConfiguration( );
extern int32_t Ranging_ActivateTransmitConfiguration(float frequencyInGhz);

// From mss_main.c
extern int32_t Ranging_dataPathConfig (uint16_t rxPrn);
extern Ranging_MSS_MCB    gMmwMssMCB;

/* ----------------------------------------------------------------------------------------------------------------- *
 *                                                  Local Variables
 * ----------------------------------------------------------------------------------------------------------------- */

char errorString[128];
char logString[128];

// Define generic behavior that occurs whenever we receive an invalid event
void Service_Null_Message( uint16_t message_id )
{
    System_printf( "NULL msg %u %s received for state %u\n",
                   message_id,
                   message_to_string_table[message_id],
                   State_Machine.currentState->stateNumber );
}

// Define generic behavior that occurs whenever we receive an invalid event
void Format_Error_String( char * error )
{
    snprintf(errorString, sizeof(errorString),
             "Error:%s in state %u\n",
             error,
             State_Machine.currentState->stateNumber );
}

void Send_State_Machine_Message( uint16_t message_id )
{
    if (State_Machine.currentState->stateNumber >= STATE_TOTAL_COUNT)
    {
        // Make sure we're not in an ISR
        if(BIOS_getThreadType() == BIOS_ThreadType_Task)
        {
          System_printf( "State %u higher than STATE_TOTAL_COUNT: %u\n",
            State_Machine.currentState->stateNumber,
            STATE_TOTAL_COUNT);
        }
        else
        {
          Ranging_debugAssert (0);
        }
    }
    else if ( State_Machine.currentState->stateNumber >= STATE_TOTAL_COUNT )
    {
        // Make sure we're not in an ISR
        if(BIOS_getThreadType() == BIOS_ThreadType_Task)
        {
          System_printf( "State %u higher than STATE_TOTAL_COUNT: %u\n",
            State_Machine.currentState->stateNumber,
            STATE_TOTAL_COUNT);
        }
        else
        {
          Ranging_debugAssert (0);
        }
    }
    else
    {
        // Make sure we're not in an ISR
        if(BIOS_getThreadType() == BIOS_ThreadType_Task)
        {
//          System_printf( "State %u: %s sent msg %u %s\n",
//            State_Machine.currentState->stateNumber,
//            state_to_string_table[State_Machine.currentState->stateNumber],
//                         message_id,
//                         message_to_string_table[message_id] );
        }
    }

    MsgObj msg;
    msg.id = message_id;

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
//    System_printf( "End\t%s\n",
//                   state_to_string_table[p_stateInfo->stateNumber] );
//    CLI_write ("End\t%s\n",
//               state_to_string_table[p_stateInfo->stateNumber]);

    if(p_stateInfo->stateMachine->uartHandle != NULL)
    {
        snprintf(logString,
                 sizeof(logString),
                 "End\t%s\n",
                 state_to_string_table[p_stateInfo->stateNumber]);

        UART_writePolling (p_stateInfo->stateMachine->uartHandle,
                           (uint8_t*)&logString,
                           strlen(logString));
    }
}

void Entering_State( State_Information_Ptr_t p_stateInfo )
{
  // If we are in a global phase, make sure it knows our current task
  if( p_stateInfo->stateNumber == STATE_COMPLETED ||
          p_stateInfo->stateNumber == STATE_FAILED ||
          p_stateInfo->stateNumber == STATE_CANCELLED )
  {
      p_stateInfo->stateNumber = p_stateInfo->previousStateInfo_ptr->stateNumber;
  }
  p_stateInfo->timesEntered++;

//  System_printf( "Start\t%s\t%u\n",
//                 state_to_string_table[p_stateInfo->stateNumber],
//                 p_stateInfo->timesEntered);
//  CLI_write ("Start\t%s\t%u\n",
//             state_to_string_table[p_stateInfo->stateNumber],
//             p_stateInfo->timesEntered);

  if(p_stateInfo->stateMachine->uartHandle != NULL)
  {
      snprintf(logString,
               sizeof(logString),
               "Start\t%s\t%u\n",
               state_to_string_table[p_stateInfo->stateNumber],
               p_stateInfo->timesEntered);

      UART_writePolling (p_stateInfo->stateMachine->uartHandle,
                         (uint8_t*)&logString,
                         strlen(logString));
  }
}

void SM_Func_Initialization( State_Information_Ptr_t p_stateInfo )
{
    System_printf("State_Machine_Initialization\n");

    //////////////////////////////////////////////////////////////////
    // CONFIGURE SENSOR
    if ( Ranging_SetBaseConfiguration( ) )
    {
        Format_Error_String("Ranging_SetBaseConfiguration");
        Send_State_Machine_Message( SM_MSG_FAILED );
        return;
    }

    if ( Ranging_CreateReceiveConfiguration(p_stateInfo->rxFrequencyInGhz) )
    {
        Format_Error_String("Ranging_CreateReceiveConfiguration");
        Send_State_Machine_Message( SM_MSG_FAILED );
        return;
    }

    if ( Ranging_CreateTransmitConfiguration(p_stateInfo->txFrequencyInGhz, p_stateInfo->goldCodeNumBits, p_stateInfo->txPrn ) )
    {
        Format_Error_String("Ranging_CreateTransmitConfiguration");
        Send_State_Machine_Message( SM_MSG_FAILED );
        return;
    }

    //////////////////////////////////////////////////////////////////
    // OPEN SENSOR
    if (gMmwMssMCB.sensorState == Ranging_SensorState_INIT)
    {
        if(Ranging_openSensor(true))
        {
            Format_Error_String("Ranging_openSensor");
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

}

void SM_Func_Continue_Rx( State_Information_Ptr_t p_stateInfo )
{
    // No operation - standing by
}

void SM_Func_Cfg_Tx( State_Information_Ptr_t p_stateInfo )
{
    if ( Ranging_CreateTransmitConfiguration(p_stateInfo->txFrequencyInGhz, p_stateInfo->goldCodeNumBits, p_stateInfo->txPrn ) )
    {
        Format_Error_String("Ranging_CreateTransmitConfiguration");
        Send_State_Machine_Message( SM_MSG_FAILED );
    }
}

void SM_Func_Activate_Tx_Cfg( State_Information_Ptr_t p_stateInfo )
{
    if ( Ranging_ActivateTransmitConfiguration( p_stateInfo->txFrequencyInGhz ) )
    {
        Format_Error_String("Ranging_ActivateReceiveConfiguration");
        Send_State_Machine_Message( SM_MSG_FAILED );
    }
}

void SM_Func_Tx_Start_Code( State_Information_Ptr_t p_stateInfo )
{
    System_printf("TX Start Code\n");
}

void SM_Func_Tx_Response_Code( State_Information_Ptr_t p_stateInfo )
{
    System_printf("TX Response Code\n");
}

void SM_Func_Cfg_Rx( State_Information_Ptr_t p_stateInfo )
{
    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        Format_Error_String("Sensor already started");
        Send_State_Machine_Message( SM_MSG_FAILED );
        return;
    }
    if ( Ranging_CreateReceiveConfiguration(p_stateInfo->rxFrequencyInGhz) )
    {
        Format_Error_String("Ranging_CreateReceiveConfiguration");
        Send_State_Machine_Message( SM_MSG_FAILED );
    }
}

// Activate the frame that contains the receive profile and receive chirp
void SM_Func_Activate_Rx_Cfg( State_Information_Ptr_t p_stateInfo )
{
    System_printf("Activate RX\n");
    if ( Ranging_ActivateReceiveConfiguration( ) )
    {
        Format_Error_String("Ranging_ActivateReceiveConfiguration");
        Send_State_Machine_Message( SM_MSG_FAILED );
    }
}

// Configure the sensor through mmwavelink
// Configure the datapath running on the DSP core
// Start the sensor
void SM_Func_Rx( State_Information_Ptr_t p_stateInfo )
{
    int32_t     errCode = 0;
    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        Format_Error_String("Sensor already started");
        Send_State_Machine_Message( SM_MSG_FAILED );
        return;
    }

    if (gMmwMssMCB.sensorState == Ranging_SensorState_INIT)
    {
        Format_Error_String("Sensor not opened");
        Send_State_Machine_Message( SM_MSG_FAILED );
        return;
    }


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
    // CONFIGURE SENSOR
    if (MMWave_config (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.ctrlCfg, &errCode) < 0)
    {
        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error: Report the error */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        System_printf ("Error: mmWave Config failed [Error code: %d Subsystem: %d]\n",
                        mmWaveErrorCode, subsysErrorCode);
        Format_Error_String("Error MMWave_config");
        Send_State_Machine_Message( SM_MSG_FAILED );
        return;
    }
    else
    {
        if(Ranging_dataPathConfig(p_stateInfo->rxPrn))
        {
            Format_Error_String("Error Ranging_dataPathConfig");
            Send_State_Machine_Message( SM_MSG_FAILED );
            return;
        }
    }

    //////////////////////////////////////////////////////////////////
    // START SENSOR
    // Starts the Data Path Module (DPM) running on the DSP core
    // Waits for it to start, then starts the RF Module with MMWave_start
    if(Ranging_startSensor())
    {
        Format_Error_String("Ranging_startSensor");
        Send_State_Machine_Message( SM_MSG_FAILED );
        return;
    }

    gMmwMssMCB.sensorState = Ranging_SensorState_STARTED;
}

void SM_Func_Rx_Response_Code( State_Information_Ptr_t p_stateInfo )
{
    System_printf("RX Response Code\n");
}

void SM_Func_Task_Completed_Successfully( State_Information_Ptr_t p_stateInfo )
{
    System_printf("Task Complete\n");
}

void SM_Func_Task_Failed( State_Information_Ptr_t p_stateInfo )
{
    System_printf("**********FAILURE\n");
    System_printf(errorString);
    System_printf("**********FAILURE\n");
    Send_State_Machine_Message( SM_MSG_COMPLETED );
}

void SM_Func_Task_Cancelled( State_Information_Ptr_t p_stateInfo )
{
    System_printf("**********Cancelled\n");
}
