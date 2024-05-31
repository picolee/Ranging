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
#include <xdc/runtime/System.h>
#include <shared/ranging_mailbox.h>
#include <inc/ranging_dss.h>
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
    uint8_t processNextTimeSlot = 0;

    // Initialize the precision timer
    // It will be used to trigger the beginning of the next time slot at precise times
    timerInitialization();

    /* wait for new message and process all the messages received from the peer */
    while(1)
    {
        Semaphore_pend(g_semaphoreHandle, BIOS_WAIT_FOREVER);

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
                    break;
                }

                case CMD_DSS_TO_START_SENSOR_AT_NEXT_TIMESLOT:
                {
                    processNextTimeSlot = 1;
                    memcpy(&gMmwDssMCB.nextTimeslot, &message.timeSlot, sizeof(rangingTimeSlot_t));
                    break;
                }

                case SET_CURRENT_TIMESLOT:
                {
                    memcpy(&gMmwDssMCB.currentTimeslot, &message.timeSlot, sizeof(rangingTimeSlot_t));
                    break;
                }

                case SET_NEXT_TIMESLOT:
                {
                    memcpy(&gMmwDssMCB.nextTimeslot, &message.timeSlot, sizeof(rangingTimeSlot_t));
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

        // Did we receive an update to the next time slot?
        if(processNextTimeSlot)
        {
            if(gMmwDssMCB.nextTimeslot.slotType != SLOT_TYPE_NO_OP)
            {
                // Launch the timer to execute at the next time slot's beginning
                launchSensorAtTargetTime(gMmwDssMCB.nextTimeslot.slotStartTSCL, gMmwDssMCB.nextTimeslot.slotStartTSCH);
            }

            //
            processNextTimeSlot = 0;
        }

    }
}


