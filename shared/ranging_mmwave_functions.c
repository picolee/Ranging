/*
 * ranging_mmwave_functions.c
 * versions of the mmwave functions instantiated here to modify for our use case
 *
 *  Created on: Jun 7, 2024
 *      Author: LeeLemay
 */



#include <shared/ranging_mmwave_structures.h>

extern Mailbox_MCB gMailboxMCB;
extern MMWave_Dualcore    gMMWave_DualcoreMCB;

#ifdef SUBSYS_DSS
#pragma SET_CODE_SECTION(".l1pcode")
extern Ranging_DSS_MCB    gMmwDssMCB;
#else
extern Ranging_MSS_MCB    gMmwMssMCB;
#endif

/*
 *  ======== Mailbox_write ========
 */
int32_t Mailbox_write_internal(Mbox_Handle handle, const uint8_t *buffer, uint32_t size)
{
    Mailbox_Driver*     driver;
    SemaphoreP_Status   status;
    int32_t             retVal = 0;
    uint32_t            header = 0;
    uintptr_t           key;

    driver = (Mailbox_Driver*)handle;

    /* Sanity Check: Validate the arguments */
    if ((size == 0) || (size > MAILBOX_DATA_BUFFER_SIZE) || (buffer == NULL) || (handle == NULL) || (driver->hwCfg == NULL))
    {
        /* Error: Invalid Arguments */
        DebugP_log4 ("MAILBOX: Mailbox_write Error! Invalid param. Size=%d Buffer=(%p) handle=(%p) hwCfgPtr=(%p)\n", size, buffer, handle, driver->hwCfg);
        retVal = MAILBOX_EINVAL;
        goto exit;
    }

    if((driver->remoteEndpoint != MAILBOX_TYPE_BSS) && (driver->cfg.chType == MAILBOX_CHTYPE_MULTI))
    {
        /* Critical Section Protection*/
        key = HwiP_disable();
        if(gMailboxMCB.mssDssWriteChIDInUse == MAILBOX_UNUSED_CHANNEL_ID)
        {
            /* Mark that the TX mailbox is now in use*/
            gMailboxMCB.mssDssWriteChIDInUse = driver->cfg.chId;
            /* Release the critical section: */
            HwiP_restore(key);
        }
        else
        {
            /* Error: TX mailbox is being used by another mailbox instance*/
            DebugP_log2 ("MAILBOX: Mailbox_write Error! handle=(%p). Write attempt with TX box in use by channel ID %d\n",driver, gMailboxMCB.mssDssWriteChIDInUse);
            retVal = MAILBOX_ECHINUSE;
            /* Release the critical section: */
            HwiP_restore(key);
            goto exit;
        }
    }

    if(driver->txBoxStatus == MAILBOX_TX_BOX_FULL)
    {
        /* Error: TX mailbox is full, can not write new message until acknowledge is received from remote endpoint */
        /* Note that this should take care that the DMA has been completed as well because this flag is cleaned only after
           copy is done */
        DebugP_log1 ("MAILBOX: Mailbox_write Error! handle=(%p). Write attempt with txBoxStatus == MAILBOX_TX_BOX_FULL\n", handle);
        retVal = MAILBOX_ETXFULL;
        goto exit;
    }

    /* A write operation is starting, need to set TXbox flag to full to block any other write to this instance of mailbox*/
    driver->txBoxStatus = MAILBOX_TX_BOX_FULL;

    /* Copy data from application buffer to mailbox buffer */
    if(driver->cfg.dataTransferMode == MAILBOX_DATA_TRANSFER_MEMCPY)
    {
        if((driver->remoteEndpoint != MAILBOX_TYPE_BSS) && (driver->cfg.chType == MAILBOX_CHTYPE_MULTI))
        {
            /*Write internal header*/
            header = driver->cfg.chId;
            memcpy ((void *)(driver->hwCfg)->baseLocalToRemote.data, (void *)(&header), sizeof(header));
            /*Write message. Need to account for internal header size*/
            memcpy ((void *)((uint8_t *)((driver->hwCfg)->baseLocalToRemote.data) + MAILBOX_MULTI_CH_HEADER_SIZE), (const void *)buffer, size);
        }
        else
        {
            /*Write message.*/
            memcpy ((void *)(driver->hwCfg)->baseLocalToRemote.data, (const void *)buffer, size);
        }

        #ifdef SUBSYS_MSS
        MEM_BARRIER();
        #endif
    }
    else
    {
        DebugP_log0 ("MAILBOX: Mailbox_write Error! Only memcpy dataTransferMode is supported\n");
        retVal = MAILBOX_EINVALCFG;
        goto exit;
    }

    /* Store handle of instance in case this is a MSS/DSS driver instance*/
    if(driver->remoteEndpoint != MAILBOX_TYPE_BSS)
    {
        gMailboxMCB.lastMsgSentHandle = handle;
    }

    /* Trigger "mailbox full" interrupt to remote endpoint*/
    ((driver->hwCfg)->baseLocalToRemote.reg)->INT_TRIG = CSL_FINSR (((driver->hwCfg)->baseLocalToRemote.reg)->INT_TRIG, 0U, 0U, 1U);

    /* Next action depends on the mailbox write mode*/
    if(driver->cfg.writeMode == MAILBOX_MODE_BLOCKING)
    {
        /* Pend on semaphore until acknowledge ("mailbox_empty" interrupt) from remote endpoint is received*/
        status = SemaphoreP_pend (driver->writeSem, driver->cfg.writeTimeout);
        if (status == SemaphoreP_TIMEOUT)
        {
            /* Set error code */
            retVal = MAILBOX_ETXACKTIMEDOUT;

            /* Report the error condition: */
            DebugP_log2 ("MAILBOX:(%p) Write acknowledge timed out. Ack was never received. Number of received TX messages = %d.\n",
                         driver, driver->txCount);
        }
    }

    /* If write is blocking mode and semaphore did not timeout, write succeeded and ack received.
       If write is polling mode and we reached this point, write was done but not sure if ack has been received. */
    if( ((driver->cfg.writeMode == MAILBOX_MODE_BLOCKING) && (status != SemaphoreP_TIMEOUT)) ||
        (driver->cfg.writeMode == MAILBOX_MODE_POLLING) )
    {
        /* Increment TX count */
        driver->txCount++;

        /* Set return value */
        retVal = (int32_t)size;
    }

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to handle the link stop for Dual core.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the mmWave MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_deviceStopFxn_internal(const MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    MMWave_Msg      message;
    int32_t         retVal = 0;
    uint32_t        stopMessageSize;

    /* Determine the execution mode? Only in the cooperative mode do we need to send
     * a message to the peer domain. */
    if (ptrMMWaveMCB->initCfg.executionMode == MMWave_ExecutionMode_COOPERATIVE)
    {
        /* Populate the message: */
        message.header.id     = MMWave_PayloadId_STOP;
        message.header.length = 0U;

        /* Compute the total size of the message which is to be sent across to the remote peer */
        stopMessageSize = sizeof(MMWave_MsgHeader) + message.header.length;

        /* Send the stop message to the peer domain: The peer mailbox handle is a critical resource
         * which is shared across multiple threads. */
        SemaphoreP_pend (gMMWave_DualcoreMCB.mailboxSemHandle, SemaphoreP_WAIT_FOREVER);
        retVal = Mailbox_write_internal (gMMWave_DualcoreMCB.peerMailbox, (const uint8_t*)&message, stopMessageSize);
        SemaphoreP_post (gMMWave_DualcoreMCB.mailboxSemHandle);
        if (retVal == (int32_t)stopMessageSize)
        {
            /* Success: Stop message has been successfully sent to the peer execution
             * domain. Setup the return value appropriately. */
            retVal = 0;
        }
        else
        {
            /* Error: Unable to send out the stop message to the peer execution domain.
             * Setup the return values and error code */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EMSG, retVal);
            retVal   = MINUS_ONE;
        }
    }
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to stop the mmWave control module after the
 *      configuration has been applied.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_start
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_stop_internal (MMWave_Handle mmWaveHandle, int32_t* errCode)
{
    MMWave_MCB*         ptrMMWaveMCB;
    int32_t             retVal;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if (ptrMMWaveMCB == NULL)
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /****************************************************************************************
     * Sanity Check:
     *  - Validate the prerequisites
     ****************************************************************************************/
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_STARTED) == 0U)
    {
        /* Error: Invalid usage the module should be started before it can be stopped. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Stop the mmWave link */
    retVal = MMWave_stopLink (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: Unable to stop the link; error code is setup already */


        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error/Warning: Unable to stop the mmWave module */
        MMWave_decodeError (*errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        if (errorLevel == MMWave_ErrorLevel_ERROR)
        {
            /* Error: Display the error message: */
            System_printf ("Error: mmWave Stop failed [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);

            /* Not expected */
            Ranging_debugAssert(0);
        }
        else
        {
            /* Warning: This is treated as a successful stop. */
            System_printf ("mmWave Stop error ignored [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);
        }

        goto exit;
    }

    /* Invoke the SOC specific function to handle the stop */
    retVal = MMWave_deviceStopFxn_internal (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: SOC specific function could not be stopped. Error code is already
         * setup */


        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error/Warning: Unable to stop the mmWave module */
        MMWave_decodeError (*errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        if (errorLevel == MMWave_ErrorLevel_ERROR)
        {
            /* Error: Display the error message: */
            System_printf ("Error: mmWave Stop failed [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);

            /* Not expected */
            Ranging_debugAssert(0);
        }
        else
        {
            /* Warning: This is treated as a successful stop. */
            System_printf ("mmWave Stop error ignored [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);
        }

        goto exit;
    }

    /* The module has been stopped successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status & (~(uint32_t)MMWAVE_STATUS_STARTED);

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to trigger the Front end to stop generating chirps.
 *
 *  @retval
 *      Not Applicable.
 */
int32_t Ranging_mmWaveCtrlStop (void)
{
    int32_t                 errCode = 0;

    DebugP_log0("App: Issuing MMWave_stop\n");

    /* Stop the mmWave module: */
#ifdef SUBSYS_DSS
    if (MMWave_stop_internal (gMmwDssMCB.ctrlHandle, &errCode) < 0)
#else
    if (MMWave_stop_internal (gMmwMssMCB.ctrlHandle, &errCode) < 0)
#endif
    {
        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error/Warning: Unable to stop the mmWave module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        if (errorLevel == MMWave_ErrorLevel_ERROR)
        {
            /* Error: Display the error message: */
            System_printf ("Error: mmWave Stop failed [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);

            /* Not expected */
            Ranging_debugAssert(0);
        }
        else
        {
            /* Warning: This is treated as a successful stop. */
            System_printf ("mmWave Stop error ignored [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);
        }
    }

    return errCode;
}


/**
 *  @b Description
 *  @n
 *      The function is used to handle the link start for Dual core.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the mmWave MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_deviceStartFxn_internal(const MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    MMWave_Msg      message;
    int32_t         retVal = 0;
    uint32_t        startMessageSize;

    /* Determine the execution mode? Only in the cooperative mode do we need to send
     * a message to the peer domain. */
    if (ptrMMWaveMCB->initCfg.executionMode == MMWave_ExecutionMode_COOPERATIVE)
    {
        /* Populate the message: */
        message.header.id     = MMWave_PayloadId_START;
        message.header.length = sizeof(MMWave_CalibrationCfg);
        memcpy ((void *)&message.u.calibrationCfg, (void*)&ptrMMWaveMCB->calibrationCfg, sizeof(MMWave_CalibrationCfg));

        /* Compute the total size of the message which is to be sent across to the remote peer */
        startMessageSize = sizeof(MMWave_MsgHeader) + message.header.length;

        /* Send the start message to the peer domain: The peer mailbox handle is a critical resource
         * which is shared across multiple threads. */
        SemaphoreP_pend (gMMWave_DualcoreMCB.mailboxSemHandle, SemaphoreP_WAIT_FOREVER);
        retVal = Mailbox_write (gMMWave_DualcoreMCB.peerMailbox, (const uint8_t*)&message, startMessageSize);
        SemaphoreP_post (gMMWave_DualcoreMCB.mailboxSemHandle);
        if (retVal == (int32_t)startMessageSize)
        {
            /* Success: Start message has been successfully sent to the peer execution
             * domain. Setup the return value appropriately. */
            retVal = 0;
        }
        else
        {
            /* Error: Unable to send out the start message to the peer execution domain.
             * Setup the return values and error code */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EMSG, retVal);
            retVal   = MINUS_ONE;
        }
    }
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to start the mmWave control module after the
 *      configuration has been applied.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[in]  ptrCalibrationCfg
 *      Pointer to the calibration configuration
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @pre
 *      MMWave_sync
 *  @pre
 *      MMWave_open
 *  @pre
 *      MMWave_config (Only in full configuration mode)
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */

int32_t MMWave_start_part_one_internal (MMWave_Handle mmWaveHandle, const MMWave_CalibrationCfg* ptrCalibrationCfg, int32_t* errCode)
{
    MMWave_MCB*     ptrMMWaveMCB;
    int32_t         retVal =1;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if ((ptrMMWaveMCB == NULL) || (ptrCalibrationCfg == NULL))
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /****************************************************************************************
     * Sanity Check:
     *  - Validate the prerequisites
     ****************************************************************************************/
    if (ptrMMWaveMCB->initCfg.cfgMode == MMWave_ConfigurationMode_FULL)
    {
        /* Full Configuration Mode: Ensure that the application has configured the mmWave module
         * Only then can we start the module. */
        if (((ptrMMWaveMCB->status & MMWAVE_STATUS_SYNCHRONIZED) == 0U)    ||
            ((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED)       == 0U)    ||
            ((ptrMMWaveMCB->status & MMWAVE_STATUS_CONFIGURED)   == 0U))
        {
            /* Error: Invalid usage the module should be synchronized before it can be started. */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            goto exit;
        }

        /* Sanity Check: Validate the DFE output mode. This should always match in the FULL configuration mode. */
        if (ptrMMWaveMCB->dfeDataOutputMode != ptrCalibrationCfg->dfeDataOutputMode)
        {
            /* Error: Invalid argument. */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            goto exit;
        }
    }
    else
    {
        /* Minimal Configuration Mode: Application should have opened and synchronized the mmWave module
         * Configuration of the mmWave link is the responsibility of the application using the link API */
        if (((ptrMMWaveMCB->status & MMWAVE_STATUS_SYNCHRONIZED) == 0U) ||
            ((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED)       == 0U))
        {
            /* Error: Invalid usage the module should be synchronized before it can be started. */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            goto exit;
        }

        /* Initialize the DFE Output mode: */
        ptrMMWaveMCB->dfeDataOutputMode = ptrCalibrationCfg->dfeDataOutputMode;
    }

    /* Sanity Check: Ensure that the module has not already been started */
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_STARTED) == MMWAVE_STATUS_STARTED)
    {
        /* Error: Invalid usage the module should be stopped before it can be started again. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Copy over the calibration configuration: */
    memcpy ((void*)&ptrMMWaveMCB->calibrationCfg, (const void*)ptrCalibrationCfg, sizeof(MMWave_CalibrationCfg));

    // Removing this means that the MSS will not receive the callback for Ranging_mssMmwaveStartCallbackFxn
    // It's removed because it accesses a semaphore from an ISR, which causes a crash
    // SOC specific start: We need to notify the peer domain before the real time starts. //
    retVal = MMWave_deviceStartFxn_internal (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        // Error: SOC Start failed; error code is already setup //
        goto exit;
    }

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}
int32_t MMWave_start_part_two_internal (MMWave_Handle mmWaveHandle, const MMWave_CalibrationCfg* ptrCalibrationCfg, int32_t* errCode)
{
    MMWave_MCB*     ptrMMWaveMCB;
    int32_t         retVal =1;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Start the mmWave link: */
    retVal = MMWave_startLink (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: Unable to start the link; error code is already setup */
        goto exit;
    }

    /* The module has been started successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status | MMWAVE_STATUS_STARTED;

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

int32_t MMWave_start_internal (MMWave_Handle mmWaveHandle, const MMWave_CalibrationCfg* ptrCalibrationCfg, int32_t* errCode)
{
    MMWave_MCB*     ptrMMWaveMCB;
    int32_t         retVal =1;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if ((ptrMMWaveMCB == NULL) || (ptrCalibrationCfg == NULL))
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /****************************************************************************************
     * Sanity Check:
     *  - Validate the prerequisites
     ****************************************************************************************/
    if (ptrMMWaveMCB->initCfg.cfgMode == MMWave_ConfigurationMode_FULL)
    {
        /* Full Configuration Mode: Ensure that the application has configured the mmWave module
         * Only then can we start the module. */
        if (((ptrMMWaveMCB->status & MMWAVE_STATUS_SYNCHRONIZED) == 0U)    ||
            ((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED)       == 0U)    ||
            ((ptrMMWaveMCB->status & MMWAVE_STATUS_CONFIGURED)   == 0U))
        {
            /* Error: Invalid usage the module should be synchronized before it can be started. */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            goto exit;
        }

        /* Sanity Check: Validate the DFE output mode. This should always match in the FULL configuration mode. */
        if (ptrMMWaveMCB->dfeDataOutputMode != ptrCalibrationCfg->dfeDataOutputMode)
        {
            /* Error: Invalid argument. */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            goto exit;
        }
    }
    else
    {
        /* Minimal Configuration Mode: Application should have opened and synchronized the mmWave module
         * Configuration of the mmWave link is the responsibility of the application using the link API */
        if (((ptrMMWaveMCB->status & MMWAVE_STATUS_SYNCHRONIZED) == 0U) ||
            ((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED)       == 0U))
        {
            /* Error: Invalid usage the module should be synchronized before it can be started. */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            goto exit;
        }

        /* Initialize the DFE Output mode: */
        ptrMMWaveMCB->dfeDataOutputMode = ptrCalibrationCfg->dfeDataOutputMode;
    }

    /* Sanity Check: Ensure that the module has not already been started */
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_STARTED) == MMWAVE_STATUS_STARTED)
    {
        /* Error: Invalid usage the module should be stopped before it can be started again. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Copy over the calibration configuration: */
    memcpy ((void*)&ptrMMWaveMCB->calibrationCfg, (const void*)ptrCalibrationCfg, sizeof(MMWave_CalibrationCfg));

    // Removing this means that the MSS will not receive the callback for Ranging_mssMmwaveStartCallbackFxn
    // It's removed because it accesses a semaphore from an ISR, which causes a crash
    // SOC specific start: We need to notify the peer domain before the real time starts. //
    retVal = MMWave_deviceStartFxn_internal (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        // Error: SOC Start failed; error code is already setup //
        goto exit;
    }

    /* Start the mmWave link: */
    retVal = MMWave_startLink (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: Unable to start the link; error code is already setup */
        goto exit;
    }

    /* The module has been started successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status | MMWAVE_STATUS_STARTED;

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function can be used by the application to get the chirp handle
 *      at the specified index. If the index exceeds the number of chirps
 *      configured the function will fail with the error code.
 *
 *  @param[in]  profileHandle
 *      Handle to the profile
 *  @param[in]  chirpIndex
 *      Chirp Index for which the handle is needed. Set to 1 to get the
 *      first chirp index etc
 *  @param[out] chirpHandle
 *      Populated chirp handle
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getChirpHandle_internal
(
    MMWave_ProfileHandle    profileHandle,
    uint32_t                chirpIndex,
    MMWave_ChirpHandle*     chirpHandle,
    int32_t*                errCode
)
{
    MMWave_Profile*     ptrMMWaveProfile;
    MMWave_Chirp*       ptrMMWaveChirp;
    uint32_t            index  = 1U;
    int32_t             retVal = MINUS_ONE;
    int32_t             endProcessing = 0;

    /* Initialize the error code: */
    *errCode     = 0;

    /* Sanity Check: Validate the arguments */
    if ((profileHandle == NULL) || (chirpHandle == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the Profile: */
    ptrMMWaveProfile = (MMWave_Profile*)profileHandle;

    /* Sanity Check: Profiles are always linked to the control module */
    DebugP_assert (ptrMMWaveProfile->ptrMMWaveMCB != NULL);

    /* Sanity Check: These API are available only in FULL configuration mode */
    DebugP_assert (ptrMMWaveProfile->ptrMMWaveMCB->initCfg.cfgMode == MMWave_ConfigurationMode_FULL);

    /* Initialize the chirp handle */
    *chirpHandle = NULL;

    /* Critical Section Enter: Protect the 'Chirp List' */
    SemaphoreP_pend (ptrMMWaveProfile->ptrMMWaveMCB->cfgSemHandle, SemaphoreP_WAIT_FOREVER);

    /* Get the head of the chirp list: */
    ptrMMWaveChirp = (MMWave_Chirp*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveProfile->ptrChirpList);
    while (endProcessing == 0)
    {
        /* Have we reached the end of the list? */
        if (ptrMMWaveChirp == NULL)
        {
            /* YES: Control comes here indicates that the chirp index specified exceeds the
             * configured number of chirps. We are done with the processing */
            *errCode      = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            endProcessing = 1;
        }
        else
        {
            /* Is this what we are looking for? */
            if (index == chirpIndex)
            {
                /* YES: Setup the chirp handle. */
                *chirpHandle  = (MMWave_ChirpHandle)ptrMMWaveChirp;
                retVal        = 0;
                endProcessing = 1;
            }

            /* Get the next element: */
            index = index + 1U;
            ptrMMWaveChirp = (MMWave_Chirp*)MMWave_listGetNext ((MMWave_ListNode*)ptrMMWaveChirp);
        }
    }

    /* Critical Section Exit: */
    SemaphoreP_post (ptrMMWaveProfile->ptrMMWaveMCB->cfgSemHandle);

exit:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to get the chirp configuration given
 *      the chirp handle
 *
 *  @param[in]  chirpHandle
 *      Handle to the chirp
 *  @param[out] ptrChirpCfg
 *      Pointer to the chirp configuration populated by the API
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getChirpCfg_internal
(
    MMWave_ChirpHandle  chirpHandle,
    rlChirpCfg_t*       ptrChirpCfg,
    int32_t*            errCode
)
{
    MMWave_Chirp*   ptrMMWaveChirp;
    int32_t         retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((chirpHandle == NULL) || (ptrChirpCfg == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the Chirp: */
    ptrMMWaveChirp = (MMWave_Chirp*)chirpHandle;

    /* Chirps are always linked to profiles and profiles to the mmWave control module. */
    DebugP_assert (ptrMMWaveChirp->ptrMMWaveProfile != NULL);
    DebugP_assert (ptrMMWaveChirp->ptrMMWaveProfile->ptrMMWaveMCB != NULL);

    /* Sanity Check: These API are available only in FULL configuration mode */
    DebugP_assert (ptrMMWaveChirp->ptrMMWaveProfile->ptrMMWaveMCB->initCfg.cfgMode == MMWave_ConfigurationMode_FULL);

    /* Copy over the configuration: */
    memcpy ((void*)ptrChirpCfg, (void*)&ptrMMWaveChirp->chirpCfg, sizeof(rlChirpCfg_t));

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure BPM
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the MCB
 *  @param[in]  ptrControlCfg
 *      Pointer to the control config
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_configBPM_internal
(
    MMWave_MCB*         ptrMMWaveMCB,
    MMWave_CtrlCfg*     ptrControlCfg,
    int32_t*            errCode
)
{
    int32_t                 retVal;
    uint32_t                numBpmChirps = 0;
    uint32_t                index;
    MMWave_BpmChirpHandle   bpmChirpHandle;
    rlBpmChirpCfg_t**       bpmPtrArray;
    rlBpmCommonCfg_t        bpmCommonCfg;
    uint32_t                arraySize;
    MMWave_BpmChirp*        ptrMMWaveBpmChirp;


    /* Get the number of BPM chirps configured */
    if (MMWave_getNumBpmChirp ((MMWave_Handle)ptrMMWaveMCB, &numBpmChirps, errCode) < 0)
    {
        /* Error: Unable to get the number of BPM chirps. Error code is already setup */
        retVal = MINUS_ONE;
        goto end;
    }

    if(numBpmChirps == 0)
    {
        /* No BPM chirp configured. Nothing to be done.*/
        retVal = 0;
        goto end;
    }

    arraySize = numBpmChirps * sizeof(rlBpmChirpCfg_t*);

    /* Allocate array to store pointers to BPM configurations*/
    bpmPtrArray = (rlBpmChirpCfg_t**) MemoryP_ctrlAlloc (arraySize, 0);

    if (bpmPtrArray == NULL)
    {
        /* Error: Out of memory */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ENOMEM, 0);
        retVal   = MINUS_ONE;
        goto end;
    }

    /* Initialize the allocated memory for the chirp: */
    memset ((void*)bpmPtrArray, 0, arraySize);

    /* Select source of BPM pattern to be from BPM chirp cfg defined in bpmChirpCfg*/
    memset ((void *)&bpmCommonCfg, 0, sizeof(rlBpmCommonCfg_t));
    bpmCommonCfg.mode.b2SrcSel = 0U;

    /* Set the BPM common config */
    retVal = rlSetBpmCommonConfig(RL_DEVICE_MAP_INTERNAL_BSS, &bpmCommonCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Setting the BPM configuration failed */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ECOMMONBPMCFG, retVal);
        retVal   = MINUS_ONE;
        goto end;
    }

    /* Cycle through all the BPM configurations and populate array. */
    for (index = 1U; index <= numBpmChirps; index++)
    {
        /* Get the Handle associated to the specified index */
        if (MMWave_getBpmChirpHandle ((MMWave_Handle)ptrMMWaveMCB, index, &bpmChirpHandle, errCode) < 0)
        {
            /* Error: Unable to get the handle. Error code is already setup */
            retVal = MINUS_ONE;
            goto end;
        }

        /* Populate the BPM cfg array. Note that index starts from 1 and array starts from zero. */
        ptrMMWaveBpmChirp = (MMWave_BpmChirp*)bpmChirpHandle;
        bpmPtrArray[index-1] = (rlBpmChirpCfg_t*)(&ptrMMWaveBpmChirp->bpmChirp);
    }

    /* Set the BPM chirp configuration in the mmWave link */
    retVal = rlSetMultiBpmChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS,
                                     (rlUInt16_t)numBpmChirps,
                                     bpmPtrArray);

    /* Free the memory used by the config array) */
    MemoryP_ctrlFree ((void *)bpmPtrArray, arraySize);

    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Setting the BPM configuration failed */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EBPMCFG, retVal);
        retVal   = MINUS_ONE;
        goto end;
    }

end:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the mmWave link with the specified profile
 *      and corresponding chirp configuration. The following order is preserved in
 *      the function:
 *          - Profile configuration
 *          - Chirp configuration
 *
 *  @param[in]  ptrProfileHandle
 *      Pointer to profile handle
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MMWave_configureProfileChirp_internal (MMWave_ProfileHandle* ptrProfileHandle, int32_t* errCode)
{
    int32_t                 retVal;
    int32_t                 index;
    rlProfileCfg_t          profileCfg;
    MMWave_ChirpHandle      chirpHandle;
    rlChirpCfg_t            chirpCfg;
    uint32_t                numChirps;
    uint32_t                chirpIndex;

    /* Cycle through all the profile(s) which have been specified. */
    for (index = 0U; index < MMWAVE_MAX_PROFILE; index++)
    {
        /* Do we have a valid profile? */
        if (ptrProfileHandle[index] == NULL)
        {
            /* NO: Skip to the next profile */
            continue;
        }

        /* YES: Get the profile configuration */
        if (MMWave_getProfileCfg (ptrProfileHandle[index], &profileCfg, errCode) < 0)
        {
            /* Error: Unable to get the profile configuration. Setup the return value */
            retVal = MINUS_ONE;
            goto end;
        }

        /* Configure the profile using the mmWave Link API */
        retVal = rlSetProfileConfig (RL_DEVICE_MAP_INTERNAL_BSS, 1U, &profileCfg);
        if (retVal != RL_RET_CODE_OK)
        {
            /* Error: Setting the profile configuration failed */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EPROFILECFG, retVal);
            retVal   = MINUS_ONE;
            goto end;
        }

        /* Get the number of chirps configured and attached to the profile: */
        if (MMWave_getNumChirps (ptrProfileHandle[index], &numChirps, errCode) < 0)
        {
            /* Error: Unable to get the number of chirps. Error code is already setup */
            retVal = MINUS_ONE;
            goto end;
        }

        /* For the profile; Cycle through all the chirps and configure them. */
        for (chirpIndex = 1U; chirpIndex <= numChirps; chirpIndex++)
        {
            /* Get the Chirp Handle associated at the specified index */
            if (MMWave_getChirpHandle_internal (ptrProfileHandle[index], chirpIndex, &chirpHandle, errCode) < 0)
            {
                /* Error: Unable to get the chirp handle. Error code is already setup */
                retVal = MINUS_ONE;
                goto end;
            }

            /* Get the chirp configuration: */
            if (MMWave_getChirpCfg_internal (chirpHandle, &chirpCfg, errCode) < 0)
            {
                /* Error: Unable to get the chirp configuration. Error code is already setup */
                retVal = MINUS_ONE;
                goto end;
            }

            /* Set the chirp configuration in the mmWave link */
            retVal = rlSetChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, 1U, &chirpCfg);
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Setting the chirp configuration failed */
                *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ECHIRPCFG, retVal);
                retVal   = MINUS_ONE;
                goto end;
            }
        }
    }

    /* Control comes here implies that the profile & chirp was configured successfully */
    retVal = 0;

end:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to configure the mmWave link with the supplied
 *      configuration
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[in]  ptrControlCfg
 *      Pointer to the control configuration
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_configLink_internal
(
    MMWave_MCB*         ptrMMWaveMCB,
    MMWave_CtrlCfg*     ptrControlCfg,
    int32_t*            errCode
)
{
    int32_t retVal;

    /* Determine the DFE Output mode? */
    switch (ptrControlCfg->dfeDataOutputMode)
    {
        case MMWave_DFEDataOutputMode_FRAME:
        {
            /**************************************************************************
             * Frame Mode:
             * Order of operations as specified by the mmWave Link are
             *  - Profile configuration
             *  - Chirp configuration
             *  - Frame configuration
             **************************************************************************/
            retVal = MMWave_configureProfileChirp_internal (&ptrControlCfg->u.frameCfg.profileHandle[0], errCode);
            if (retVal < 0)
            {
                /* Error: Unable to configure the profile & chirps. Error code is already setup */
                goto end;
            }

            if (MMWave_configBPM_internal (ptrMMWaveMCB, ptrControlCfg, errCode) < 0)
            {
                /* Error: Unable to configure BPM chirps. Error code is already setup */
                retVal = -1;
                goto end;
            }

            /* Set the frame configuration: */
            retVal = rlSetFrameConfig(RL_DEVICE_MAP_INTERNAL_BSS, &ptrControlCfg->u.frameCfg.frameCfg);
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Setting the frame configuration failed */
                *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
                retVal   = MINUS_ONE;
                goto end;
            }
            break;
        }
        case MMWave_DFEDataOutputMode_CONTINUOUS:
        {
            /**************************************************************************
             * Continuous Mode:
             **************************************************************************/
            retVal = rlSetContModeConfig (RL_DEVICE_MAP_INTERNAL_BSS, &ptrControlCfg->u.continuousModeCfg.cfg);
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Unable to setup the continuous mode */
                *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ECONTMODECFG, retVal);
                retVal   = MINUS_ONE;
                goto end;
            }
            break;
        }
        case MMWave_DFEDataOutputMode_ADVANCED_FRAME:
        {
            /**************************************************************************
             * Advanced Frame Configuration:
             * Order of operations as specified by the mmWave Link are
             *  - Profile configuration
             *  - Chirp configuration
             *  - Advanced Frame configuration
             **************************************************************************/
            retVal = MMWave_configureProfileChirp_internal (&ptrControlCfg->u.advancedFrameCfg.profileHandle[0], errCode);
            if (retVal < 0)
            {
                /* Error: Unable to configure the profile & chirps. Error code is already setup */
                goto end;
            }

            if (MMWave_configBPM_internal (ptrMMWaveMCB, ptrControlCfg, errCode) < 0)
            {
                /* Error: Unable to configure BPM chirps. Error code is already setup */
                retVal = -1;
                goto end;
            }

            /* Set the advanced frame configuration: */
            retVal = rlSetAdvFrameConfig(RL_DEVICE_MAP_INTERNAL_BSS, &ptrControlCfg->u.advancedFrameCfg.frameCfg);
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Setting the frame configuration failed */
                *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
                retVal   = MINUS_ONE;
                goto end;
            }
            break;
        }
        default:
        {
            /* Error: This should never occur and the user seems to have ignored a warning. */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            retVal   = MINUS_ONE;
            goto end;
        }
    }

    /* Set the return value to be success. */
    retVal = 0;

end:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is the exported API which is used to configure the mmWave.
 *      This is only applicable in the full configuration mode.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[in]  ptrControlCfg
 *      Pointer to the control configuration
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @pre
 *      MMWave_sync
 *  @pre
 *      MMWave_open
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_config_internal (MMWave_Handle mmWaveHandle, MMWave_CtrlCfg* ptrControlCfg, int32_t* errCode)
{
    MMWave_MCB* ptrMMWaveMCB;
    int32_t     retVal;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if ((ptrMMWaveMCB == NULL) || (ptrControlCfg == NULL))
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /****************************************************************************************
     * Sanity Check:
     *  - Validate the prerequisites
     ****************************************************************************************/
    if (ptrMMWaveMCB->initCfg.cfgMode == MMWave_ConfigurationMode_FULL)
    {
        /* Full Configuration Mode: Ensure that the application has opened the mmWave module. */
        if (((ptrMMWaveMCB->status & MMWAVE_STATUS_SYNCHRONIZED) == 0U) ||
            ((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED)       == 0U))
        {
            /* Error: Invalid usage the module should be synchronized before it can be started. */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            goto exit;
        }

        /* Initialize the DFE Output mode: */
        ptrMMWaveMCB->dfeDataOutputMode = ptrControlCfg->dfeDataOutputMode;
    }
    else
    {
        /* Minimal Configuration Mode: Application is responsible for the configuration of the BSS using
         * the mmWave Link API. This is av invalid use case */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Configure the link: */
    if (MMWave_configLink_internal (ptrMMWaveMCB, ptrControlCfg, errCode) < 0)
    {
        /* Error: Unable to configure the link; error code is already setup. */
        goto exit;
    }

    /* SOC specific configuration: */
    retVal = MMWave_deviceCfgFxn (ptrMMWaveMCB, ptrControlCfg, errCode);
    if (retVal < 0)
    {
        /* Error: SOC configuration failed; error code is already setup */
        goto exit;
    }

    /* The module has been configured successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status | MMWAVE_STATUS_CONFIGURED;

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

