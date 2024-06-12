/*
 * ranging_mailbox.c
 * Provides interprocess communication between two cores - MSS, DSS, and BSS
 *
 *  Created on: May 24, 2024
 *      Author: LeeLemay
 */

#ifdef SUBSYS_DSS
#pragma SET_CODE_SECTION(".l1pcode")
#endif

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <xdc/runtime/System.h>
#include "ranging_mailbox.h"
#include <shared/ranging_mmwave_structures.h>
#include <shared/ranging_queue.h>

///////////////////////////////////////////////////
// TYPEDEFS
///////////////////////////////////////////////////

///////////////////////////////////////////////////
// GLOBALS
///////////////////////////////////////////////////

extern MMWave_Dualcore  gMMWave_DualcoreMCB;

Mbox_Handle             g_mboxHandle;

static Semaphore_Handle g_writeSemaphore;
Semaphore_Handle        g_readSemaphore;

// Pre-allocated message pool
#define MAX_QUEUE_SIZE 5
static rangingQueue_t g_writeQueue;

////////////////////////////////////////////////////
//  FUNCTIONS
////////////////////////////////////////////////////

/**
* @b Description
* @n
* Function to send a message to peer through Mailbox virtual channel
*
* @param[in] message
* Pointer to the MMW demo message.
*
* @retval
* Success - 0
* Fail < -1
*/
static int32_t mboxWrite(Ranging_MSS_DSS_Message * message)
{
//    int32_t retVal = -1;
//
//    SemaphoreP_pend (gMMWave_DualcoreMCB.mailboxSemHandle, SemaphoreP_WAIT_FOREVER);
//    retVal = Mailbox_write (g_mboxHandle, (uint8_t*)message, sizeof(Ranging_MSS_DSS_Message));
//    SemaphoreP_post (gMMWave_DualcoreMCB.mailboxSemHandle);
//    if (retVal == sizeof(Ranging_MSS_DSS_Message))
//    {
//        retVal = 0;
//    }
//    return retVal;


    if (rangingQueueEnqueue(&g_writeQueue, message))
    {
        Semaphore_post(g_writeSemaphore);
        return 0;
    }

    return -1;
}

void cmdDssToStartSensorNow(rangingTimeSlot_Ptr_t p_timeSlot)
{
    Ranging_MSS_DSS_Message message;
    memcpy(&message.data.timeSlot, p_timeSlot, sizeof(rangingTimeSlot_t));
    message.messageId = CMD_DSS_TO_START_SENSOR_NOW;
    mboxWrite(&message);
}

void cmdDssToMsgMssAtNextTimeslot()
{
    Ranging_MSS_DSS_Message message;
    message.messageId = CMD_DSS_TO_MSG_MSS_AT_NEXT_TIMESLOT;
    mboxWrite(&message);
}

void cmdDssToStartSensorAtNextTimeslot(rangingTimeSlot_Ptr_t p_timeSlot)
{
    Ranging_MSS_DSS_Message message;
    memcpy(&message.data.timeSlot, p_timeSlot, sizeof(rangingTimeSlot_t));
    message.messageId = CMD_DSS_TO_START_SENSOR_AT_NEXT_TIMESLOT ;
    mboxWrite(&message);
}


void cmdDssToStartSensorAtSpecificTxTime(rangingTimeSlot_Ptr_t   p_timeSlot)
{
    Ranging_MSS_DSS_Message message;
    memcpy(&message.data.timeSlot, p_timeSlot, sizeof(rangingTimeSlot_t));
    message.messageId = CMD_DSS_TO_START_SENSOR_AT_SPECIFIC_TX_TIME;
    mboxWrite(&message);
}

void sendCfgDataToDSS()
{
    Ranging_MSS_DSS_Message message;
    message.messageId = MSS_SENDS_CFG_DATA_TO_DSS;
    mboxWrite(&message);
}

void setNextTimeSlotOnDss(rangingTimeSlot_Ptr_t   p_timeSlot)
{
    Ranging_MSS_DSS_Message message;
    memcpy(&message.data.timeSlot, p_timeSlot, sizeof(rangingTimeSlot_t));
    message.messageId = SET_NEXT_TIMESLOT;
    mboxWrite(&message);
}

void dssReportsResult(DPC_Ranging_Data_t * p_rangingData)
{
    Ranging_MSS_DSS_Message message;
    memcpy(&message.data.rangingData, p_rangingData, sizeof(DPC_Ranging_Data_t));
    message.messageId = DSS_REPORTS_RESULT;
    mboxWrite(&message);
}

void dssReportsSuccess()
{
    Ranging_MSS_DSS_Message message;
    message.messageId = DSS_REPORTS_SUCCESS;
    mboxWrite(&message);
}

void dssReportsFailure()
{
    Ranging_MSS_DSS_Message message;
    message.messageId = DSS_REPORTS_FAILURE;
    mboxWrite(&message);
}

void dssReportsTimeslotStart()
{
    Ranging_MSS_DSS_Message message;
    message.messageId = DSS_REPORTS_NEXT_TIMESLOT_STARTED;
    mboxWrite(&message);
}

void dssReportsSensorStart()
{
    Ranging_MSS_DSS_Message message;
    message.messageId = DSS_REPORTS_SENSOR_STARTED;
    mboxWrite(&message);
}

void dssSendStringToMss(const char *string)
{
    Ranging_MSS_DSS_Message message;
    message.messageId = DSS_SEND_STRING_MESSAGE;
    strncpy(message.data.stringData, string, sizeof(message.data.stringData) - 1);
    message.data.stringData[sizeof(message.data.stringData) - 1] = '\0'; // Ensure null termination
    mboxWrite(&message);
}

/**
* @b Description
* @n
* This function is a callback funciton that invoked when a message is received from the peer.
*
* @param[in] handle
* Handle to the Mailbox on which data was received
* @param[in] peer
* Peer from which data was received

* @retval
* Not Applicable.
*/
static void mbxCallback( Mbox_Handle handle, Mailbox_Type peer )
{
    // Message has been received from the peer endpoint.
    // Wakeup the mmWave thread to process
    // the received message.
    Semaphore_post (g_readSemaphore);
}

// Function for the mailbox write task
void mailboxWriteTask(UArg arg0, UArg arg1) {
    Ranging_MSS_DSS_Message message;

    while (1)
    {
        // Wait for a message to be available in the queue
        Semaphore_pend(g_writeSemaphore, BIOS_WAIT_FOREVER);

        // Dequeue the message
        if (rangingQueueDequeue(&g_writeQueue, &message))
        {
            // Perform the mailbox write
            if (Mailbox_write(g_mboxHandle, (uint8_t *)&message, sizeof(Ranging_MSS_DSS_Message)) != sizeof(Ranging_MSS_DSS_Message))
            {
                System_printf("Error: Mailbox write failed\n");
            }
        }
    }
}

void initializeMailboxWithRemote(uint8_t taskPriority,
                                 Mailbox_Type localMailboxType,
                                 Mailbox_Type remoteMailboxType,
                                 Task_Handle* readTask,
                                 ti_sysbios_knl_Task_FuncPtr taskFunction,
                                 Task_Handle* writeTask)
{
    Task_Params         taskParams;
    Mailbox_Config      mbxCfg;
    int32_t             errCode;
    Semaphore_Params    semParams;

    /* Initialize the Mailbox */
    Mailbox_init(localMailboxType);
    errCode = Mailbox_Config_init(&mbxCfg);
    if(errCode < 0)
    {
        System_printf ("Error: MSS mbox config failed\n",errCode);
        return;
    }
    mbxCfg.writeMode        = MAILBOX_MODE_BLOCKING;
    mbxCfg.readMode         = MAILBOX_MODE_CALLBACK;
    mbxCfg.readCallback     = mbxCallback;
    mbxCfg.chType           = MAILBOX_CHTYPE_MULTI;
    mbxCfg.chId             = MAILBOX_CH_ID_0;
    g_mboxHandle            = Mailbox_open(remoteMailboxType, &mbxCfg, &errCode);

    if((g_mboxHandle == NULL) || (errCode != 0))
    {
        System_printf ("Error: MSS mbox open failed\n",errCode);
    }

    Semaphore_Params_init(&semParams);
    semParams.mode      = Semaphore_Mode_BINARY;
    g_readSemaphore     = Semaphore_create(0, &semParams, NULL);
    g_writeSemaphore    = Semaphore_create(0, &semParams, NULL);

    // Initialize the custom queues
    rangingQueueInit(&g_writeQueue, MAX_QUEUE_SIZE);

    // Create the mailbox read task
    Task_Params_init(&taskParams);
    taskParams.priority             = taskPriority;
    taskParams.stackSize            = 4 * 1024;
    (*readTask)                     = Task_create(taskFunction, &taskParams, NULL);

    // Create the mailbox write task
    taskParams.priority             = taskPriority;
    taskParams.stackSize            = 4 * 1024;
    (*writeTask)                    = Task_create(mailboxWriteTask, &taskParams, NULL);
}
