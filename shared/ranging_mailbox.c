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

///////////////////////////////////////////////////
// GLOBALS
///////////////////////////////////////////////////

Semaphore_Handle g_semaphoreHandle;
Mbox_Handle      g_mboxHandle;

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
    int32_t retVal = -1;

    retVal = Mailbox_write (g_mboxHandle, (uint8_t*)message, sizeof(Ranging_MSS_DSS_Message));
    if (retVal == sizeof(Ranging_MSS_DSS_Message))
    {
        retVal = 0;
    }
    return retVal;
}

void cmdDssToStartSensorNow()
{
    Ranging_MSS_DSS_Message message;
    message.messageId = CMD_DSS_TO_START_SENSOR_NOW;
    mboxWrite(&message);
}

void cmdDssToMsgMssAtNextTimeslot()
{
    Ranging_MSS_DSS_Message message;
    message.messageId = CMD_DSS_TO_START_SENSOR_AT_NEXT_TIMESLOT;
    mboxWrite(&message);
}

void cmdDssToStartSensorAtNextTimeslot()
{
    Ranging_MSS_DSS_Message message;
    message.messageId = CMD_DSS_TO_MSG_MSS_AT_NEXT_TIMESLOT;
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
    Semaphore_post (g_semaphoreHandle);
}

void initializeMailboxWithRemote(uint8_t taskPriority,
                                 Mailbox_Type localMailboxType,
                                 Mailbox_Type remoteMailboxType,
                                 Task_Handle* task,
                                 ti_sysbios_knl_Task_FuncPtr taskFunction)
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
    mbxCfg.writeMode        = MAILBOX_MODE_POLLING;
    mbxCfg.readMode         = MAILBOX_MODE_CALLBACK;
    mbxCfg.readCallback     = mbxCallback;           // Defined in ranging_mss_mailbox.c
    mbxCfg.chType           = MAILBOX_CHTYPE_MULTI;
    mbxCfg.chId             = MAILBOX_CH_ID_0;
    g_mboxHandle            = Mailbox_open(remoteMailboxType, &mbxCfg, &errCode);

    if((g_mboxHandle == NULL) || (errCode != 0))
    {
        System_printf ("Error: MSS mbox open failed\n",errCode);
    }

    Semaphore_Params_init(&semParams);
    semParams.mode      = Semaphore_Mode_BINARY;
    g_semaphoreHandle   = Semaphore_create(0, &semParams, NULL);

    Task_Params_init(&taskParams);
    taskParams.priority             = taskPriority;
    taskParams.stackSize            = 4 * 1024;
    (*task)                         = Task_create(taskFunction, &taskParams, NULL);
}
