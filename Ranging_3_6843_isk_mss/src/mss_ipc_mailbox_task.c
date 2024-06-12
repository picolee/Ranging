/*
 * ipc_mailbox_task.c
 *
 *  Created on: May 25, 2024
 *      Author: LeeLemay
 */

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <xdc/runtime/System.h>
#include <shared/ranging_mailbox.h>
#include <inc/state_machine.h>
#include <inc/ranging_mss.h>


extern Ranging_MSS_MCB    gMmwMssMCB;

/**
* @b Description
* @n
* The Task is used to handle the messages received by
* the MSS from the DSS
*
* @param[in] arg0
* arg0 of the Task. Not used
* @param[in] arg1
* arg1 of the Task. Not used
*
* @retval
* Not Applicable.
*/
void ranging_mssMboxReadTask(UArg arg0, UArg arg1)
{
    int32_t retVal;
    Ranging_MSS_DSS_Message message;
    char output_data[20];

    /* wait for new message and process all the messages received from the peer */
    while(1)
    {
        Semaphore_pend(g_readSemaphore, BIOS_WAIT_FOREVER);

        // Read the message from the peer mailbox
        retVal = Mailbox_read(g_mboxHandle, (uint8_t*)&message, sizeof(Ranging_MSS_DSS_Message));
        if (retVal < 0)
        {
            // Error: Unable to read the message. Setup the error code and return values
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
            /* Flush out the contents of the mailbox to indicate that we are done with the message. This will
            * allow us to receive another message in the mailbox while we process the received message. */
            Mailbox_readFlush (g_mboxHandle);

            /* Process the received message: */
            switch (message.messageId)
            {
                case DSS_REPORTS_SUCCESS:
                {
                    break;
                }

                case DSS_REPORTS_NEXT_TIMESLOT_STARTED:
                {
                    Send_New_Timeslot_Started_Message();

                    snprintf(output_data,
                             sizeof(output_data),
                             "MBOX:Next TSlot\r\n");

                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                       (uint8_t*)&output_data,
                                       strlen(output_data));
                    break;
                }

                case DSS_REPORTS_SENSOR_STARTED:
                {
                    snprintf(output_data,
                             sizeof(output_data),
                             "MBOX:Sensor Start\r\n");

                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                       (uint8_t*)&output_data,
                                       strlen(output_data));
                    break;
                }

                case DSS_REPORTS_RESULT:
                {
                    // rangingData could be transferred from DSS to MSS through hand shake ram to improve speed
                    memcpy(&gMmwMssMCB.rangingData, &message.data.rangingData, sizeof(DPC_Ranging_Data_t));
                    Send_Results_Available_Message();

                    snprintf(output_data,
                             sizeof(output_data),
                             "MBOX:Result Avail\r\n");

                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                       (uint8_t*)&output_data,
                                       strlen(output_data));
                    break;
                }

                case DSS_REPORTS_FAILURE:
                {
                    // Send the report to the state machine
                    Send_DSS_Reports_Failure_Message();

                    snprintf(output_data,
                             sizeof(output_data),
                             "MBOX:DSS Fail\r\n");

                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                       (uint8_t*)&output_data,
                                       strlen(output_data));

                    break;
                }

                case DSS_SEND_STRING_MESSAGE:
                {

                    snprintf(output_data,
                             sizeof(output_data),
                             "MBOX:String RX: ");

                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                       (uint8_t*)&output_data,
                                       strlen(output_data));

                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                       (uint8_t*)&message.data.stringData,
                                       strlen(message.data.stringData));
                    break;
                }

                default:
                {
                    // Message not supported
                    System_printf ("Error: unsupported Mailbox message id=%d\n", message.messageId);
                    break;
                }
            }
        }
    }
}
