/*
 * ipc_mailbox_task.c
 *
 *  Created on: May 25, 2024
 *      Author: LeeLemay
 */


#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/sysbios/knl/Event.h>
#include <xdc/runtime/System.h>
#include <shared/ranging_mailbox.h>
#include <inc/ranging_dss.h>
#include <inc/ranging_datapath.h>
#include <inc/countdown_timer.h>
#include <inc/dss_mmwave_sensor_interface.h>


#ifdef SUBSYS_DSS
#pragma SET_CODE_SECTION(".l1pcode")
#endif


extern Ranging_DSS_MCB    gMmwDssMCB;


/**
* @b Description
* @n
* The Task is used to handle the messages received
* by the DSS from the MSS
*
* @param[in] arg0
* arg0 of the Task. Not used
* @param[in] arg1
* arg1 of the Task. Not used
*
* @retval
* Not Applicable.
*/
void ranging_dssMboxReadTask(UArg arg0, UArg arg1)
{
    int32_t retVal;
    Ranging_MSS_DSS_Message message;
    uint8_t startSensorAtNextTimeSlot = 0;
    uint8_t startSensorAtSpecificTxTime = 0;
    uint8_t msgMssAtNextTimeSlot = 0;

    uint32_t targetTSCL;
    uint32_t targetTSCH;

    // Initialize the precision timer
    // It will be used to:
    // - trigger call backs to the MSS
    // - trigger sensor starts at the next time slot
    // - trigger sensor starts at other precise times
    timerInitialization( &gMmwDssMCB.precisionTimerTaskHandle );

    /* wait for new message and process all the messages received from the peer */
    while(1)
    {
        Semaphore_pend(g_readSemaphore, BIOS_WAIT_FOREVER);

        /* Read the message from the peer mailbox: We are not trying to protect the read
        * from the peer mailbox because this is only being invoked from a single thread */
        retVal = Mailbox_read(g_mboxHandle, (uint8_t*)&message, sizeof(Ranging_MSS_DSS_Message));
        if (retVal < 0)
        {
            /* Error: Unable to read the message. Setup the error code and return values */
            System_printf ("Error: Mailbox read failed [Error code %d]\n", retVal);
        }
        else if (retVal == 0)
        {
            // The semaphore was posted, indicating there is a message to read
            // but no bytes were read
            System_printf ("Error: No bytes read from mailbox\n");
        }
        else
        {
            // Flush out the contents of the mailbox to indicate that we are done with the message. This will
            // allow us to receive another message in the mailbox while we process the received message.
            Mailbox_readFlush (g_mboxHandle);

            // Process the received message:
            switch (message.messageId)
            {
                case CMD_DSS_TO_START_SENSOR_NOW:
                {
                    if(startSensor())
                    {
                        dssReportsFailure();
                    }
                    else
                    {
                        dssReportsSensorStart();
                    }
                    Event_post(gMmwDssMCB.eventHandle, RANGING_NEXT_TIMESLOT_STARTED_EVT);
                    break;
                }

                case CMD_DSS_TO_START_SENSOR_AT_NEXT_TIMESLOT:
                {
                    startSensorAtNextTimeSlot = 1;
                    memcpy(&gMmwDssMCB.nextTimeslot, &message.data.timeSlot, sizeof(rangingTimeSlot_t));
                    break;
                }

                case CMD_DSS_TO_START_SENSOR_AT_SPECIFIC_TX_TIME:
                {
                    startSensorAtSpecificTxTime = 1;
                    memcpy(&gMmwDssMCB.nextTimeslot, &message.data.timeSlot, sizeof(rangingTimeSlot_t));
                    break;
                }

                case CMD_DSS_TO_MSG_MSS_AT_NEXT_TIMESLOT:
                {
                    msgMssAtNextTimeSlot = 1;
                    memcpy(&gMmwDssMCB.nextTimeslot, &message.data.timeSlot, sizeof(rangingTimeSlot_t));
                    break;
                }

                case MSS_SENDS_CFG_DATA_TO_DSS:
                {
                    break;
                }

                case SET_CURRENT_TIMESLOT:
                {
                    memcpy(&gMmwDssMCB.currentTimeslot, &message.data.timeSlot, sizeof(rangingTimeSlot_t));
                    break;
                }

                case SET_NEXT_TIMESLOT:
                {
                    memcpy(&gMmwDssMCB.nextTimeslot, &message.data.timeSlot, sizeof(rangingTimeSlot_t));
                    Event_post(gMmwDssMCB.eventHandle, RANGING_CONFIG_EVT);
                    break;
                }

                default:
                {
                    /* Message not support */
                    System_printf ("Error: unsupported Mailbox message id=%d\n", message.messageId);
                    dssReportsFailure();
                    break;
                }
            }
        }

        // Did we receive a command to start the sensor at the next time slot?
        if(startSensorAtNextTimeSlot)
        {
            // TX
            if(     gMmwDssMCB.nextTimeslot.slotType ==  SLOT_TYPE_SYNCHRONIZATION_TX ||
                    gMmwDssMCB.nextTimeslot.slotType ==  SLOT_TYPE_RANGING_START_CODE_TX)
            {
                // Incorporate the TX delay after slot start
                targetTSCL = gMmwDssMCB.nextTimeslot.slotStartTSCL + gMmwDssMCB.nextTimeslot.transmitDelayAfterSlotStartsDSPCycles;
                targetTSCH = gMmwDssMCB.nextTimeslot.slotStartTSCH;

                // Check for roll over
                if(targetTSCL < gMmwDssMCB.nextTimeslot.slotStartTSCL)
                {
                    targetTSCH += 1;
                }

                launchSensorAtTargetTime(targetTSCL, targetTSCH);
                Event_post(gMmwDssMCB.eventHandle, RANGING_CONFIG_EVT);
            }

            else if (gMmwDssMCB.nextTimeslot.slotType ==  SLOT_TYPE_RANGING_RESPONSE_CODE_TX)
            {
                // SLOT_TYPE_RANGING_RESPONSE_CODE_TX should cause a startSensorAtSpecificTxTime process
                dssReportsFailure();
            }

            // RX
            else
            {
                launchSensorAtTargetTime(gMmwDssMCB.nextTimeslot.slotStartTSCL, gMmwDssMCB.nextTimeslot.slotStartTSCH);
                Event_post(gMmwDssMCB.eventHandle, RANGING_CONFIG_EVT);
            }
            startSensorAtNextTimeSlot = 0;
        }

        // Did we receive a command to TX at a specific time?
        if(startSensorAtSpecificTxTime)
        {
            if( gMmwDssMCB.nextTimeslot.slotType !=  SLOT_TYPE_RANGING_RESPONSE_CODE_TX )
            {
                dssReportsFailure();
            }
            else
            {
                launchSensorAtTargetTime(gMmwDssMCB.nextTimeslot.txResponseStartTSCL, gMmwDssMCB.nextTimeslot.txResponseStartTSCH);
            }
            Event_post(gMmwDssMCB.eventHandle, RANGING_CONFIG_EVT);
            startSensorAtSpecificTxTime = 0;
        }

        // Did we receive a command to wake up the MSS at the next time slot?
        if(msgMssAtNextTimeSlot)
        {
            msgMssAtTargetTime(gMmwDssMCB.nextTimeslot.slotStartTSCL, gMmwDssMCB.nextTimeslot.slotStartTSCH);
            msgMssAtNextTimeSlot = 0;
        }
    }
}


