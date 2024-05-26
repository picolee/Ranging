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

    /* wait for new message and process all the messages received from the peer */
    while(1)
    {
        Semaphore_pend(g_semaphoreHandle, BIOS_WAIT_FOREVER);

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
                    // Send the report to the state machine
                    Send_DSS_Reports_Success_Message();
                    break;
                }

                case DSS_REPORTS_FAILURE:
                {
                    // Send the report to the state machine
                    Send_DSS_Reports_Failure_Message();
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
