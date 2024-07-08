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
timeLowHighRegisters_t g_MSSTime;
extern uint32_t         pingStart;
extern uint32_t         pingTime;


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
    Ranging_MSS_DSS_Message_t message;
    uint32_t targetTSCL;
    uint32_t targetTSCH;
    char output_data[128];
    uint32_t timestamp;

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
        retVal = Mailbox_read(g_mboxHandle, (uint8_t*)&message, sizeof(Ranging_MSS_DSS_Message_t));
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


            g_MSSTime.timeLow = message.messageCreatimeTime.timeLow;
            g_MSSTime.timeHigh = message.messageCreatimeTime.timeHigh;

            // Process the received message:
            switch (message.messageId)
            {
                case CMD_DSS_TO_START_SENSOR_NOW:
                {
                    /*
                     * When we have SYNC_IN working, this will trigger SYNC_IN
                     * For now, let the MSS perform the sensor start
                     * When the task receives the RANGING_NEXT_TIMESLOT_STARTED_EVT,
                     * it will call dssReportsTimeslotStart();
                    if(startSensor())
                    {
                        dssReportsFailure();
                    }
                    else
                    {
                        dssReportsSensorStart();
                    }
                    */
                    Event_post(gMmwDssMCB.eventHandle, RANGING_NEXT_TIMESLOT_STARTED_EVT);
                    //dssSendStringToMss("CMD_DSS_TO_START_SENSOR_NOW\r\n");
                    break;
                }

                case CMD_DSS_TO_START_SENSOR_AT_NEXT_TIMESLOT:
                {
                    // TX
                    if (gMmwDssMCB.nextTimeslot.slotType ==  SLOT_TYPE_RANGING_RESPONSE_CODE_TX)
                    {
                        // SLOT_TYPE_RANGING_RESPONSE_CODE_TX should cause a startSensorAtSpecificTxTime process
                        dssReportsFailure();
                    }

                    else if(    gMmwDssMCB.nextTimeslot.slotType ==  SLOT_TYPE_SYNCHRONIZATION_TX ||
                                gMmwDssMCB.nextTimeslot.slotType ==  SLOT_TYPE_RANGING_START_CODE_TX)
                    {
                        // Incorporate the TX delay after slot start
                        targetTSCL = gMmwDssMCB.nextTimeslot.slotStart.timeLow + gMmwDssMCB.nextTimeslot.transmitDelayAfterSlotStartsDSPCycles;
                        targetTSCH = gMmwDssMCB.nextTimeslot.slotStart.timeHigh;

                        // Check for roll over
                        if(targetTSCL < gMmwDssMCB.nextTimeslot.slotStart.timeLow)
                        {
                            targetTSCH += 1;
                        }

                        launchSensorAtTargetTime(targetTSCL, targetTSCH);
                    }

                    // RX
                    else
                    {
                        launchSensorAtTargetTime(gMmwDssMCB.nextTimeslot.slotStart.timeLow, gMmwDssMCB.nextTimeslot.slotStart.timeHigh);
                    }
                    //dssSendStringToMss("CMD_DSS_TO_START_SENSOR_AT_NEXT_TIMESLOT\r\n");
                    break;
                }

                case CMD_DSS_TO_MSG_MSS_AT_NEXT_TIMESLOT:
                {
                    msgMssAtTargetTime(gMmwDssMCB.nextTimeslot.slotStart.timeLow, gMmwDssMCB.nextTimeslot.slotStart.timeHigh);
                    //dssSendStringToMss("CMD_DSS_TO_MSG_MSS_AT_NEXT_TIMESLOT\r\n");
                    break;
                }

                case MSS_SENDS_CFG_DATA_TO_DSS:
                {
                    break;
                }

                case SET_CURRENT_TIMESLOT:
                {
                    memcpy(&gMmwDssMCB.currentTimeslot, &message.data.timeSlot, sizeof(rangingTimeSlot_t));
                    //dssSendStringToMss("SET_CURRENT_TIMESLOT\r\n");
                    break;
                }

                case SET_NEXT_TIMESLOT:
                {
                    memcpy(&gMmwDssMCB.nextTimeslot, &message.data.timeSlot, sizeof(rangingTimeSlot_t));
                    Event_post(gMmwDssMCB.eventHandle, RANGING_CONFIG_EVT);
                    //dssSendStringToMss("SET_NEXT_TIMESLOT\r\n");
                    break;
                }

                case PING:
                {
                    ack();
                    break;
                }

                case ACK:
                {
                    pingTime = Cycleprofiler_getTimeStamp() - pingStart;
                    timestamp = Cycleprofiler_getTimeStamp();

                    snprintf(output_data,
                             sizeof(output_data),
                             "Ping Start: %u End: %u Duration: %u, DSS Time: %u\r\n",
                             pingStart,
                             timestamp,
                             pingTime,
                             g_MSSTime.timeLow);
                    dssSendStringToMss(&output_data);
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
    }
}


