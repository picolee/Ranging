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
#include <ti/utils/cycleprofiler/cycle_profiler.h>

///////////////////////////////////////////////////
// TYPEDEFS
///////////////////////////////////////////////////

///////////////////////////////////////////////////
// GLOBALS
///////////////////////////////////////////////////

extern MMWave_Dualcore  gMMWave_DualcoreMCB;

Mbox_Handle             g_mboxHandle;
uint32_t                pingStart;
uint32_t                pingTime;

static Semaphore_Handle g_writeSemaphore;
Semaphore_Handle        g_readSemaphore;

// Pre-allocated message pool
#define MAX_QUEUE_SIZE 20
static rangingQueue_t   g_writeQueue;

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
static int32_t mboxWrite(Ranging_MSS_DSS_Message_t * message)
{

    if (rangingQueueEnqueue(&g_writeQueue, message))
    {
        Semaphore_post(g_writeSemaphore);
        return 0;
    }

    return -1;
}

void populateMessage(Ranging_MSS_DSS_Message_t * message, ipcMessageId_t messageId)
{
    message->messageCreatimeTime.timeLow = Cycleprofiler_getTimeStamp();
    message->messageId = messageId;
    message->messageCreatimeTime.timeLow = Cycleprofiler_getTimeStamp();
#ifdef SUBSYS_DSS
    message->messageCreatimeTime.timeHigh = TSCH;
#else
    message->messageCreatimeTime.timeHigh = 0;
#endif
}

void cmdDssToStartSensorNow()
{
    Ranging_MSS_DSS_Message_t message;
    populateMessage(&message, CMD_DSS_TO_START_SENSOR_NOW);
    if(mboxWrite(&message))
    {
        System_printf("Error writing cmdDssToStartSensorNow\n");
    }
}

void cmdDssToMsgMssAtNextTimeslot()
{
    Ranging_MSS_DSS_Message_t message;
    populateMessage(&message, CMD_DSS_TO_MSG_MSS_AT_NEXT_TIMESLOT);
    if(mboxWrite(&message))
    {
        System_printf("Error writing cmdDssToMsgMssAtNextTimeslot\n");
    }
}

void cmdDssToStartSensorAtNextTimeslot()
{
    Ranging_MSS_DSS_Message_t message;
    populateMessage(&message, CMD_DSS_TO_START_SENSOR_AT_NEXT_TIMESLOT);
    if(mboxWrite(&message))
    {
        System_printf("Error writing cmdDssToStartSensorAtNextTimeslot\n");
    }
}

void sendCfgDataToDSS()
{
    Ranging_MSS_DSS_Message_t message;
    populateMessage(&message, MSS_SENDS_CFG_DATA_TO_DSS);
    if(mboxWrite(&message))
    {
        System_printf("Error writing sendCfgDataToDSS\n");
    }
}

void setNextTimeSlotOnDss(rangingTimeSlot_Ptr_t   p_timeSlot)
{
    Ranging_MSS_DSS_Message_t message;
    populateMessage(&message, SET_NEXT_TIMESLOT);
    memcpy(&message.data.timeSlot, p_timeSlot, sizeof(rangingTimeSlot_t));
    if(mboxWrite(&message))
    {
        System_printf("Error writing setNextTimeSlotOnDss\n");
    }
}

void dssReportsResult(DPC_Ranging_Data_t * p_rangingData)
{
    Ranging_MSS_DSS_Message_t message;
    populateMessage(&message, DSS_REPORTS_RESULT);
    memcpy(&message.data.rangingData, p_rangingData, sizeof(DPC_Ranging_Data_t));
    if(mboxWrite(&message))
    {
        System_printf("Error writing dssReportsResult\n");
    }
}

void dssReportsSuccess()
{
    Ranging_MSS_DSS_Message_t message;
    populateMessage(&message, DSS_REPORTS_SUCCESS);
    if(mboxWrite(&message))
    {
        System_printf("Error writing dssReportsSuccess\n");
    }
}

void dssReportsFailure()
{
    Ranging_MSS_DSS_Message_t message;
    populateMessage(&message, DSS_REPORTS_FAILURE);
    if(mboxWrite(&message))
    {
        System_printf("Error writing dssReportsFailure\n");
    }
}

void dssReportsTimeslotStart()
{
    Ranging_MSS_DSS_Message_t message;
    populateMessage(&message, DSS_REPORTS_NEXT_TIMESLOT_STARTED);
    if(mboxWrite(&message))
    {
        System_printf("Error writing dssReportsTimeslotStart\n");
    }
}

void dssReportsSensorStart()
{
    Ranging_MSS_DSS_Message_t message;
    populateMessage(&message, DSS_REPORTS_SENSOR_STARTED);
    if(mboxWrite(&message))
    {
        System_printf("Error writing dssReportsSensorStart\n");
    }
}

void dssSendStringToMss(const char *string)
{
    Ranging_MSS_DSS_Message_t message;
    populateMessage(&message, DSS_SEND_STRING_MESSAGE);
    strncpy(message.data.stringData, string, sizeof(message.data.stringData) - 1);
    message.data.stringData[sizeof(message.data.stringData) - 1] = '\0'; // Ensure null termination
    if(mboxWrite(&message))
    {
        System_printf("Error writing dssSendStringToMss\n");
    }
}


void ping()
{
    Ranging_MSS_DSS_Message_t message;
    populateMessage(&message, PING);
    pingStart = Cycleprofiler_getTimeStamp();
    if(mboxWrite(&message))
    {
        System_printf("Error writing ping\n");
    }
}

void ack()
{
    Ranging_MSS_DSS_Message_t message;
    populateMessage(&message, ACK);
    if(mboxWrite(&message))
    {
        System_printf("Error writing ack\n");
    }
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
    Ranging_MSS_DSS_Message_t message;

    while (1)
    {
        // Wait for a message to be available in the queue
        Semaphore_pend(g_writeSemaphore, BIOS_WAIT_FOREVER);

        // Dequeue the message
        if (rangingQueueDequeue(&g_writeQueue, &message))
        {
            // Perform the mailbox write
            if (Mailbox_write(g_mboxHandle, (uint8_t *)&message, sizeof(Ranging_MSS_DSS_Message_t)) != sizeof(Ranging_MSS_DSS_Message_t))
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
