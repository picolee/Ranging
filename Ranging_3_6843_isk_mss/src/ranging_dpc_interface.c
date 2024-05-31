/*
 * ranging_dpc_interface.c
 *
 *  Created on: May 13, 2024
 *      Author: LeeLemay
 */


#include <stdint.h>

#include <xdc/runtime/System.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

#include <inc/ranging_config.h>
#include <inc/ranging_mss.h>
#include <inc/ranging_rfparser.h>
#include <inc/ranging_dpc.h>
#include <inc/state_machine.h>
#include <inc/ranging_dpc_interface.h>


extern Ranging_MSS_MCB    gMmwMssMCB;
extern int32_t Ranging_configCQ(Ranging_SubFrameCfg *subFrameCfg,
                                uint8_t numChirpsPerChirpEvent,
                                uint8_t validProfileIdx);
extern void Ranging_transferLVDSUserData(uint8_t subFrameIndx,
                                         DPC_Ranging_ExecuteResult *dpcResults);

static uint8_t Ranging_getPrevSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames);
static uint8_t Ranging_getNextSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames);

static void Ranging_transmitProcessedOutput
(
    UART_Handle     uartHandle,
    DPC_Ranging_ExecuteResult   *result,
    Ranging_output_message_stats        *timingInfo
);

extern uint32_t gts[100];
extern uint32_t gtsIdx;

/**
 *  @b Description
 *  @n
 *      The function is used to configure the data path based on the chirp profile.
 *      After this function is executed, the data path processing will ready to go
 *      when the ADC buffer starts receiving samples corresponding to the chirps.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Ranging_dataPathConfig (uint16_t rxPrn)
{
    int32_t                         errCode;
    MMWave_CtrlCfg                  *ptrCtrlCfg;
    Ranging_DPC_ObjDet_CommonCfg    *objDetCommonCfg;
    Ranging_SubFrameCfg             *subFrameCfg;
    int8_t                          subFrameIndx;
    Ranging_RFParserOutParams       RFparserOutParams;
    DPC_Ranging_PreStartCfg         objDetPreStartDspCfg;

    /* Get data path object and control configuration */
    ptrCtrlCfg = &gMmwMssMCB.cfg.ctrlCfg;

    objDetCommonCfg = &gMmwMssMCB.objDetCommonCfg;

    /* Get RF frequency scale factor */
    gMmwMssMCB.rfFreqScaleFactor = SOC_getDeviceRFFreqScaleFactor(gMmwMssMCB.socHandle, &errCode);
    if (errCode < 0)
    {
        System_printf ("Error: Unable to get RF scale factor [Error:%d]\n", errCode);
        Ranging_debugAssert(0);
    }

    gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames =
        Ranging_RFParser_getNumSubFrames(ptrCtrlCfg);

    DebugP_log0("App: Issuing Pre-start Common Config IOCTL to DSP\n");

    /* DPC pre-start common config */
    errCode = Ranging_DPM_ioctl_blocking (gMmwMssMCB.objDetDpmHandle,
                         DPC_RANGING_IOCTL__STATIC_PRE_START_COMMON_CFG,
                         &objDetCommonCfg->preStartCommonCfg,
                         sizeof (DPC_Ranging_PreStartCommonCfg));

    if (errCode < 0)
    {
        System_printf ("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG [Error:%d]\n", errCode);
        goto exit;
    }

    /* Reason for reverse loop is that when sensor is started, the first sub-frame
     * will be active and the ADC configuration needs to be done for that sub-frame
     * before starting (ADC buf hardware does not have notion of sub-frame, it will
     * be reconfigured every sub-frame). This cannot be alternatively done by calling
     * the Ranging_ADCBufConfig function only for the first sub-frame because this is
     * a utility API that computes the rxChanOffset that is part of ADC dataProperty
     * which will be used by range DPU and therefore this computation is required for
     * all sub-frames.
     */
    for(subFrameIndx = gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames -1; subFrameIndx >= 0;
        subFrameIndx--)
    {
        subFrameCfg  = &gMmwMssMCB.subFrameCfg[subFrameIndx];

        /*****************************************************************************
         * Data path :: Algorithm Configuration
         *****************************************************************************/

        /* Parse the profile and chirp configs and get the valid number of TX Antennas */
        errCode = Ranging_RFParser_parseConfig(&RFparserOutParams, subFrameIndx,
                                         &gMmwMssMCB.cfg.openCfg, ptrCtrlCfg,
                                         &subFrameCfg->adcBufCfg,
                                         gMmwMssMCB.rfFreqScaleFactor,
                                         subFrameCfg->bpmCfg.isEnabled);

        if (errCode != 0)
        {
            System_printf ("Error: Ranging_RFParser_parseConfig [Error:%d]\n", errCode);
            goto exit;
        }

        subFrameCfg->numChirpsPerChirpEvent = RFparserOutParams.numChirpsPerChirpEvent;
        subFrameCfg->adcBufChanDataSize = RFparserOutParams.adcBufChanDataSize;
        subFrameCfg->numAdcSamples = RFparserOutParams.numAdcSamples;
        subFrameCfg->numChirpsPerSubFrame = RFparserOutParams.numChirpsPerFrame;
        subFrameCfg->numVirtualAntennas = RFparserOutParams.numVirtualAntennas;

        errCode = Ranging_ADCBufConfig(gMmwMssMCB.adcBufHandle,
                                 gMmwMssMCB.cfg.openCfg.chCfg.rxChannelEn,
                                 subFrameCfg->numChirpsPerChirpEvent,
                                 subFrameCfg->adcBufChanDataSize,
                                 &subFrameCfg->adcBufCfg,
                                 &objDetPreStartDspCfg.staticCfg.ADCBufData.dataProperty.rxChanOffset[0]);
        if (errCode < 0)
        {
            System_printf("Error: ADCBuf config failed with error[%d]\n", errCode);
            Ranging_debugAssert (0);
        }

        errCode = Ranging_configCQ(subFrameCfg, RFparserOutParams.numChirpsPerChirpEvent,
                                   RFparserOutParams.validProfileIdx);

        if (errCode < 0)
        {
            goto exit;
        }

        /* DPC pre-start config */
        {
            int32_t idx;


            /* only complex format supported */
            Ranging_debugAssert(subFrameCfg->adcBufCfg.adcFmt == 0);

            /***********************************************************************
              Pre-start preparation for objdetdsp
             ***********************************************************************/
            memset((void *)&objDetPreStartDspCfg, 0, sizeof(DPC_Ranging_PreStartCfg));
            objDetPreStartDspCfg.subFrameNum = subFrameIndx;
            objDetPreStartDspCfg.staticCfg.ADCBufData.data = (void *)SOC_XWR68XX_MSS_ADCBUF_BASE_ADDRESS;
            objDetPreStartDspCfg.staticCfg.ADCBufData.dataProperty.adcBits = 2; /* 16-bit */
            objDetPreStartDspCfg.staticCfg.ADCBufData.dataProperty.dataFmt                  = DPIF_DATAFORMAT_COMPLEX16_IMRE;
            objDetPreStartDspCfg.staticCfg.ADCBufData.dataProperty.interleave               = DPIF_RXCHAN_NON_INTERLEAVE_MODE;
            objDetPreStartDspCfg.staticCfg.ADCBufData.dataProperty.numAdcSamples            = RFparserOutParams.numAdcSamples;
            objDetPreStartDspCfg.staticCfg.ADCBufData.dataProperty.numChirpsPerChirpEvent   = RFparserOutParams.numChirpsPerChirpEvent;
            objDetPreStartDspCfg.staticCfg.ADCBufData.dataProperty.numRxAntennas            = RFparserOutParams.numRxAntennas;
            objDetPreStartDspCfg.staticCfg.ADCBufData.dataSize                              = RFparserOutParams.numRxAntennas * RFparserOutParams.numAdcSamples * sizeof(cmplx16ImRe_t);
            objDetPreStartDspCfg.staticCfg.numChirpsPerFrame                                = RFparserOutParams.numChirpsPerFrame;
            objDetPreStartDspCfg.staticCfg.numChirpsPerFrame    = RFparserOutParams.numChirpsPerFrame;
            objDetPreStartDspCfg.staticCfg.numTxAntennas        = RFparserOutParams.numTxAntennas;
            objDetPreStartDspCfg.staticCfg.numVirtualAntAzim    = RFparserOutParams.numVirtualAntAzim;
            objDetPreStartDspCfg.staticCfg.numVirtualAntElev    = RFparserOutParams.numVirtualAntElev;
            objDetPreStartDspCfg.staticCfg.numVirtualAntennas   = RFparserOutParams.numVirtualAntennas;
            objDetPreStartDspCfg.staticCfg.isBpmEnabled         = subFrameCfg->bpmCfg.isEnabled;
            objDetPreStartDspCfg.staticCfg.centerFreq           = RFparserOutParams.centerFreq;
            objDetPreStartDspCfg.staticCfg.adcSampleRate        = RFparserOutParams.adcSampleRate;
            objDetPreStartDspCfg.staticCfg.rxPrn                = rxPrn;

            for (idx = 0; idx < RFparserOutParams.numRxAntennas; idx++)
            {
                objDetPreStartDspCfg.staticCfg.rxAntOrder[idx] = RFparserOutParams.rxAntOrder[idx];
            }

            for (idx = 0; idx < RFparserOutParams.numTxAntennas; idx++)
            {
                objDetPreStartDspCfg.staticCfg.txAntOrder[idx] = RFparserOutParams.txAntOrder[idx];
            }

            // DPC running on remote core, address need to be converted
            objDetPreStartDspCfg.staticCfg.ADCBufData.data = (void *) SOC_translateAddress((uint32_t)objDetPreStartDspCfg.staticCfg.ADCBufData.data,
                                                 SOC_TranslateAddr_Dir_TO_OTHER_CPU,
                                                 &errCode);
            DebugP_assert ((uint32_t)objDetPreStartDspCfg.staticCfg.ADCBufData.data != SOC_TRANSLATEADDR_INVALID);

            /* send pre-start config */
            errCode = Ranging_DPM_ioctl_blocking (gMmwMssMCB.objDetDpmHandle,
                                 DPC_RANGING_IOCTL__STATIC_PRE_START_CFG,
                                 &objDetPreStartDspCfg,
                                 sizeof (DPC_Ranging_PreStartCfg));
            DebugP_log0("App: DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG is processed \n");

            if (errCode < 0)
            {
                System_printf ("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG [Error:%d]\n", errCode);
                goto exit;
            }
        }
    }
exit:
    return errCode;
}


/**
 *  @b Description
 *  @n
 *      Perform Data path driver open
 *
 *  @retval
 *      Not Applicable.
 */
void Ranging_dataPathOpen(void)
{
    gMmwMssMCB.adcBufHandle = Ranging_ADCBufOpen(gMmwMssMCB.socHandle);
    if(gMmwMssMCB.adcBufHandle == NULL)
    {
        Ranging_debugAssert(0);
    }
}

/**
 *  @b Description
 *  @n
 *      This function is used to start data path to handle chirps from front end.
 *
 *  @retval
 *      Not Applicable.
 */
void Ranging_dataPathStart (void)
{
    int32_t retVal;

    DebugP_log0("App: Issuing DPM_start\n");

    /* Configure HW LVDS stream for the first sub-frame that will start upon
     * start of frame */
    if (gMmwMssMCB.subFrameCfg[0].lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
    {
        Ranging_configLVDSHwData(0);
    }

    /* Start the DPM Profile: */
    if ((retVal = DPM_start(gMmwMssMCB.objDetDpmHandle)) < 0)
    {
        /* Error: Unable to start the profile */
        System_printf("Error: Unable to start the DPM [Error: %d]\n", retVal);
        Ranging_debugAssert(0);
    }

    /* Wait until start completed */
    Semaphore_pend(gMmwMssMCB.DPMstartSemHandle, BIOS_WAIT_FOREVER);

    DebugP_log0("App: DPM_start Done (post Semaphore_pend on reportFxn reporting start)\n");
}

/**
 *  @b Description
 *  @n
 *      This function is used to stop data path.
 *
 *  @retval
 *      Not Applicable.
 */
void Ranging_dataPathStop (void)
{
    int32_t retVal;

    DebugP_log0("App: Issuing DPM_stop\n");

    retVal = DPM_stop (gMmwMssMCB.objDetDpmHandle);
    if (retVal < 0)
    {
        System_printf ("DPM_stop failed[Error code %d]\n", retVal);
        Ranging_debugAssert(0);
    }
}

/**
 *  @b Description
 *  @n
 *      Function to handle frame processing results from DPC
 *
 *  @param[in] ptrResult      Pointer to DPC result
 *
 *  @retval
 *      Not Applicable.
 */
void Ranging_rangingResult
(
    DPM_Buffer  *ptrResult
)
{
    int32_t     retVal;
    DPC_Ranging_ExecuteResultExportedInfo exportInfo;
    DPC_Ranging_ExecuteResult        *dpcResults;
    Ranging_output_message_stats            *frameStats;
    volatile uint32_t                        startTime;
    uint8_t                                  nextSubFrameIdx;
    uint8_t                                  numSubFrames;
    uint8_t                                  currSubFrameIdx;
    uint8_t                                  prevSubFrameIdx;
    Ranging_SubFrameStats                    *currSubFrameStats;
    Ranging_SubFrameStats                    *prevSubFrameStats;

    /*****************************************************************
     * datapath has finished frame processing, results are reported
     *****************************************************************/

    /* Validate DPC results buffer */
    DebugP_assert (ptrResult->size[0] == sizeof(DPC_Ranging_ExecuteResult));

    /* Translate the address: */
    dpcResults = (DPC_Ranging_ExecuteResult *)SOC_translateAddress((uint32_t)ptrResult->ptrBuffer[0],
                                             SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                             &retVal);
    DebugP_assert ((uint32_t)dpcResults != SOC_TRANSLATEADDR_INVALID);

    /* Validate timing Info buffer */
    DebugP_assert (ptrResult->size[1] == sizeof(Ranging_output_message_stats));

    numSubFrames = gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames;
    currSubFrameIdx = dpcResults->subFrameIdx;
    prevSubFrameIdx = Ranging_getPrevSubFrameIndx(currSubFrameIdx, numSubFrames);
    currSubFrameStats = &gMmwMssMCB.subFrameStats[currSubFrameIdx];
    prevSubFrameStats = &gMmwMssMCB.subFrameStats[prevSubFrameIdx];

    /*****************************************************************
     * Transmit results
     *****************************************************************/
    startTime = Cycleprofiler_getTimeStamp();
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();


    /* Translate the address: */
    frameStats = (Ranging_output_message_stats *)SOC_translateAddress((uint32_t)ptrResult->ptrBuffer[1],
                                             SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                             &retVal);
    DebugP_assert ((uint32_t)frameStats != SOC_TRANSLATEADDR_INVALID);

    /* Update current frame stats */
    currSubFrameStats->outputStats.interFrameCPULoad = frameStats->interFrameCPULoad;
    currSubFrameStats->outputStats.activeFrameCPULoad= gMmwMssMCB.subFrameStats[currSubFrameIdx].outputStats.activeFrameCPULoad;
    currSubFrameStats->outputStats.interChirpProcessingMargin = frameStats->interChirpProcessingMargin;
    currSubFrameStats->outputStats.interFrameProcessingTime = frameStats->interFrameProcessingTime;
    prevSubFrameStats->outputStats.interFrameProcessingMargin = frameStats->interFrameProcessingMargin;
    currSubFrameStats->outputStats.interFrameProcessingMargin = currSubFrameStats->outputStats.interFrameProcessingMargin -
                                                         (currSubFrameStats->pendingConfigProcTime + currSubFrameStats->subFramePreparationTime);

    if (gMmwMssMCB.subFrameCfg[currSubFrameIdx].lvdsStreamCfg.dataFmt !=
             MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
    {
        /* Pend for completion of h/w session, generally this will not wait
         * because of time spent doing inter-frame processing is expected to
         * be bigger than the transmission of the h/w session */
        Semaphore_pend(gMmwMssMCB.lvdsStream.hwFrameDoneSemHandle, BIOS_WAIT_FOREVER);
    }
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();

    /* Transfer data on LVDS if s/w session is enabled for the current sub-frame */
    if(gMmwMssMCB.subFrameCfg[currSubFrameIdx].lvdsStreamCfg.isSwEnabled == 1)
    {
        Ranging_transferLVDSUserData(currSubFrameIdx, dpcResults);
    }
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();

    /* Transmit processing results for the frame */
    Ranging_transmitProcessedOutput(gMmwMssMCB.loggingUartHandle,
                                    dpcResults,
                                    &currSubFrameStats->outputStats);
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();

    /* Wait until s/w session is complete. We expect the LVDS transmission of
     * s/w session to be completed by now because the UART transmission above is slower.
     * Doing the wait immediately after starting the transmission above Ranging_transferLVDSUserData
     * will serialize the LVDS and UART transfers so it is better to do after UART
     * transmission (which is blocking call i.e UART transmission is completed when we exit
     * out of above Ranging_transmitProcessedOutput). Note we cannot replace below code
     * with check for previous sub-frame s/w session completion before the
     * Ranging_transferLVDSUserData above because we need to ensure that
     * current sub-frame/frame's contents are not being read during the
     * next sub-frame/frame transmission, presently the data that is being
     * transmitted is not double buffered to allow this */
    if(gMmwMssMCB.subFrameCfg[currSubFrameIdx].lvdsStreamCfg.isSwEnabled == 1)
    {
        /* Pend completion of s/w session, no wait is expected here */
        Semaphore_pend(gMmwMssMCB.lvdsStream.swFrameDoneSemHandle, BIOS_WAIT_FOREVER);
    }
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();

    /* Update current frame transmit time */
    currSubFrameStats->outputStats.transmitOutputTime = (Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ; /* In micro seconds */


    /*****************************************************************
     * Handle dynamic pending configuration
     * For non-advanced frame case:
     *   process all pending dynamic config commands.
     * For advanced-frame case:
     *  Process next sub-frame related pending dynamic config commands.
     *  If the next sub-frame was the first sub-frame of the frame,
     *  then process common (sub-frame independent) pending dynamic config
     *  commands.
     *****************************************************************/
    startTime = Cycleprofiler_getTimeStamp();

    nextSubFrameIdx = Ranging_getNextSubFrameIndx(currSubFrameIdx,   numSubFrames);
    if (retVal != 0)
    {
        System_printf ("Error: Executing Pending Dynamic Configuration Commands [Error code %d]\n",
                       retVal);
        Ranging_debugAssert (0);
    }
    currSubFrameStats->pendingConfigProcTime = (Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ;
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();
    /*****************************************************************
     * Prepare for subFrame switch
     *****************************************************************/
    if(numSubFrames > 1)
    {
        Ranging_SubFrameCfg  *nextSubFrameCfg;
        uint16_t dummyRxChanOffset[SYS_COMMON_NUM_RX_CHANNEL];

        startTime = Cycleprofiler_getTimeStamp();

        nextSubFrameCfg = &gMmwMssMCB.subFrameCfg[nextSubFrameIdx];

        /* Configure ADC for next sub-frame */
        retVal = Ranging_ADCBufConfig(gMmwMssMCB.adcBufHandle,
                                 gMmwMssMCB.cfg.openCfg.chCfg.rxChannelEn,
                                 nextSubFrameCfg->numChirpsPerChirpEvent,
                                 nextSubFrameCfg->adcBufChanDataSize,
                                 &nextSubFrameCfg->adcBufCfg,
                                 &dummyRxChanOffset[0]);
        if(retVal < 0)
        {
            System_printf("Error: ADCBuf config failed with error[%d]\n", retVal);
            Ranging_debugAssert (0);
        }

        /* Configure HW LVDS stream for this subframe? */
        if(nextSubFrameCfg->lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
        {
            Ranging_configLVDSHwData(nextSubFrameIdx);
        }

        currSubFrameStats->subFramePreparationTime = (Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ;
        gts[gtsIdx++]=Cycleprofiler_getTimeStamp();
    }
    else
    {
        currSubFrameStats->subFramePreparationTime = 0;
    }

    /*****************************************************************
     * Send notification to data path after results are handled
     *****************************************************************/
    /* Indicate result consumed and end of frame/sub-frame processing */
    exportInfo.subFrameIdx = currSubFrameIdx;
    retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                         DPC_RANGING_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED,
                         &exportInfo,
                         sizeof (DPC_Ranging_ExecuteResultExportedInfo));
    if (retVal < 0) {
        System_printf ("Error: DPM DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED failed [Error code %d]\n",
                       retVal);
        Ranging_debugAssert (0);
    }
    gts[gtsIdx++]=Cycleprofiler_getTimeStamp();

    if (gtsIdx>90) gtsIdx=0;
}



/**
 *  @b Description
 *  @n
 *      DPM Registered Report Handler. The DPM Module uses this registered function to notify
 *      the application about DPM reports.
 *
 *  @param[in]  reportType
 *      Report Type
 *  @param[in]  instanceId
 *      Instance Identifier which generated the report
 *  @param[in]  errCode
 *      Error code if any.
 *  @param[in] arg0
 *      Argument 0 interpreted with the report type
 *  @param[in] arg1
 *      Argument 1 interpreted with the report type
 *
 *  @retval
 *      Not Applicable.
 */
void Ranging_DPC_reportFxn
(
    DPM_Report  reportType,
    uint32_t    instanceId,
    int32_t     errCode,
    uint32_t    arg0,
    uint32_t    arg1
)
{
    /* Only errors are logged on the console: */
    if ((errCode != 0) )
    {
        /* Error: Detected log on the console and die all errors are FATAL currently. */
        System_printf ("Error: DPM Report %d received with error:%d arg0:0x%x arg1:0x%x\n",
                        reportType, errCode, arg0, arg1);
        DebugP_assert (0);
    }

    /* Processing further is based on the reports received: This is the control of the profile
     * state machine: */
    switch (reportType)
    {
        case DPM_Report_IOCTL:
        {
            /*****************************************************************
             * DPC has been configured without an error:
             * - This is an indication that the profile configuration commands
             *   went through without any issues.
             *****************************************************************/
            DebugP_log1("App: DPM Report IOCTL, command = %d\n", arg0);

            if (arg0 == DPC_RANGING_IOCTL__STATIC_PRE_START_CFG)
            {
                DPC_Ranging_PreStartCfg *cfg;
                DPC_Ranging_DPC_IOCTL_preStartCfg_memUsage *memUsage;

                cfg = (DPC_Ranging_PreStartCfg*)arg1;

                memUsage = &cfg->memUsage;

                System_printf("============ Heap Memory Stats ============\n");
                System_printf("%20s %12s %12s %12s %12s\n", " ", "Size", "Used", "Free", "DPCUsed");
                System_printf("%20s %12d %12d %12d %12d\n", "System Heap(L2)",
                              memUsage->SystemHeapTotal, memUsage->SystemHeapUsed,
                              memUsage->SystemHeapTotal - memUsage->SystemHeapUsed,
                              memUsage->SystemHeapDPCUsed);

                System_printf("%20s %12d %12d %12d\n", "L3",
                              memUsage->L3RamTotal,
                              memUsage->L3RamUsage,
                              memUsage->L3RamTotal - memUsage->L3RamUsage);

                System_printf("%20s %12d %12d %12d\n", "localRam(L2)",
                              memUsage->CoreL2RamTotal,
                              memUsage->CoreL2RamUsage,
                              memUsage->CoreL2RamTotal - memUsage->CoreL2RamUsage);

                System_printf("%20s %12d %12d %12d\n", "localRam(L1)",
                              memUsage->CoreL1RamTotal,
                              memUsage->CoreL1RamUsage,
                              memUsage->CoreL1RamTotal - memUsage->CoreL1RamUsage);
            }

            switch(arg0)
            {
                /* The following ioctls take longer time to finish. It causes DPM to queue IOCTL requests on DSS before
                 * they are handled. However DPM has limited pipe queues, hence adding sync points in demo to avoid
                 * sending too many such ioctls to DSS at a time.
                 * The semaphore blocks CLI task to wait for the response from DSS before sending the next ioctl.
                 */
                case DPC_RANGING_IOCTL__STATIC_PRE_START_COMMON_CFG:
                case DPC_RANGING_IOCTL__STATIC_PRE_START_CFG:
                    Semaphore_post(gMmwMssMCB.DPMioctlSemHandle);
                    break;
                default:
                    break;
            }
            break;
        }
        case DPM_Report_DPC_STARTED:
        {
            /*****************************************************************
             * DPC has been started without an error:
             * - notify sensor management task that DPC is started.
             *****************************************************************/
            DebugP_log0("App: DPM Report DPC Started\n");
            gMmwMssMCB.stats.dpmStartEvents++;
            Semaphore_post(gMmwMssMCB.DPMstartSemHandle);
            break;
        }
        case DPM_Report_NOTIFY_DPC_RESULT:
        {
            /*****************************************************************
             * datapath has finished frame processing, results are reported
             *****************************************************************/
            DPM_Buffer*     ptrResult;

            /* Get the result: */
            ptrResult = (DPM_Buffer*)arg0;

            Ranging_rangingResult(ptrResult);
            break;
        }
        case DPM_Report_DPC_ASSERT:
        {
            DPM_DPCAssert*  ptrAssert;

            /*****************************************************************
             * DPC Fault has been detected:
             * - This implies that the DPC has crashed.
             * - The argument0 points to the DPC assertion information
             *****************************************************************/
            ptrAssert = (DPM_DPCAssert*)arg0;
            CLI_write("Obj Det DPC Exception: %s, line %d.\n", ptrAssert->fileName,
                       ptrAssert->lineNum);
            break;
        }
        case DPM_Report_DPC_STOPPED:
        {
            /*****************************************************************
             * DPC has been stopped without an error:
             * - This implies that the DPC can either be reconfigured or
             *   restarted.
             *****************************************************************/
            DebugP_log0("App: DPM Report DPC Stopped\n");
            gMmwMssMCB.stats.dpmStopEvents++;
            /* every sensor stop should cause 2 DPM stop events due to distributed domain
               Wait for both the events before proceeding with remaining steps */
            Semaphore_post(gMmwMssMCB.DPMstopSemHandle);
            break;
        }
        case DPM_Report_DPC_INFO:
        case DPM_Report_NOTIFY_DPC_RESULT_ACKED:
        {
            /* Currently objDetDsp does not use this feature. */
            break;
        }
        default:
        {
            DebugP_assert (0);
            break;
        }
    }
    return;
}


/** @brief Transmits detection data over UART
*
*    The following data is transmitted:
*    1. Header (size = 32bytes), including "Magic word", (size = 8 bytes)
*       and including the number of TLV items
*    TLV Items:
*    2. If detectedObjects flag is 1 or 2, DPIF_PointCloudCartesian structure containing
*       X,Y,Z location and velocity for detected objects,
*       size = sizeof(DPIF_PointCloudCartesian) * number of detected objects
*    3. If detectedObjects flag is 1, DPIF_PointCloudSideInfo structure containing SNR
*       and noise for detected objects,
*       size = sizeof(DPIF_PointCloudCartesian) * number of detected objects
*    4. If logMagRange flag is set,  rangeProfile,
*       size = number of range bins * sizeof(uint16_t)
*    5. If noiseProfile flag is set,  noiseProfile,
*       size = number of range bins * sizeof(uint16_t)
*    6. If rangeAzimuthHeatMap flag is set, the zero Doppler column of the
*       range cubed matrix, size = number of Rx Azimuth virtual antennas *
*       number of chirps per frame * sizeof(uint32_t)
*    7. If rangeDopplerHeatMap flag is set, the log magnitude range-Doppler matrix,
*       size = number of range bins * number of Doppler bins * sizeof(uint16_t)
*    8. If statsInfo flag is set, the stats information
*   @param[in] uartHandle   UART driver handle
*   @param[in] result       Pointer to result from object detection DPC processing
*   @param[in] timingInfo   Pointer to timing information provided from core that runs data path
*/
static void Ranging_transmitProcessedOutput
(
    UART_Handle     uartHandle,
    DPC_Ranging_ExecuteResult   *result,
    Ranging_output_message_stats        *timingInfo
)
{
    Ranging_output_message_header header;
    Ranging_GuiMonSel   *pGuiMonSel;
    Ranging_SubFrameCfg *subFrameCfg;
    uint32_t tlvIdx = 0;
    uint32_t numPaddingBytes;
    uint32_t packetLen;
    uint8_t padding[RANGING_OUTPUT_MSG_SEGMENT_LEN];
    Ranging_output_message_tl   tl[MMWDEMO_OUTPUT_MSG_MAX];
    int32_t errCode;
    int16_t index;
    DPC_Ranging_Stats   *stats;
    DPC_Ranging_Data    *rangingData;
    Ranging_PRN_Detection_Stats* detectionStats;
    char output_data[20];
    char output_float[40];
    int16_t* pBuffer;
    static int16_t num_received = 0;

    /* Get subframe configuration */
    subFrameCfg = &gMmwMssMCB.subFrameCfg[result->subFrameIdx];

    /* Get Gui Monitor configuration */
    pGuiMonSel = &subFrameCfg->guiMonSel;

    /* Clear message header */
    memset((void *)&header, 0, sizeof(Ranging_output_message_header));

    /******************************************************************
       Send out data that is enabled, Since processing results are from DSP,
       address translation is needed for buffer pointers
    *******************************************************************/
    {
        stats = (DPC_Ranging_Stats *) SOC_translateAddress((uint32_t)result->stats,
                                                     SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                     &errCode);
        DebugP_assert ((uint32_t)stats != SOC_TRANSLATEADDR_INVALID);


        result->radarCube.data  = (void *) SOC_translateAddress((uint32_t) result->radarCube.data,
                                                     SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                     &errCode);
        DebugP_assert ((uint32_t) result->radarCube.data != SOC_TRANSLATEADDR_INVALID);

        rangingData = (DPC_Ranging_Data *) SOC_translateAddress((uint32_t) result->rangingData,
                                                     SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                     &errCode);
        DebugP_assert ((uint32_t) rangingData != SOC_TRANSLATEADDR_INVALID);

        detectionStats = &rangingData->detectionStats;

        memcpy(&gMmwMssMCB.rangingResult.detectionStats, &rangingData->detectionStats, sizeof(Ranging_PRN_Detection_Stats));

        // Update the state machine
        Send_Results_Available_Message();
    }

    // Data is arranged as:

    // ADC Data In - pHwRes->radarCube.data
    // rangingObj->radarCubebuf                = (cmplx16ImRe_t *)pHwRes->radarCube.data;
    // rangingObj->fftOfMagnitude              = (cmplx16ImRe_t *)rangingObj->radarCubebuf                  + pStaticCfg->ADCBufData.dataSize;
    // rangingObj->magnitudeData               = (cmplx16ImRe_t *)rangingObj->fftOfMagnitude                + pStaticCfg->ADCBufData.dataSize;
    // rangingObj->vectorMultiplyOfFFtedData   = (cmplx16ImRe_t *)rangingObj->magnitudeData                 + pStaticCfg->ADCBufData.dataSize;
    // rangingObj->iFftData                    = (cmplx16ImRe_t *)rangingObj->vectorMultiplyOfFFtedData     + pStaticCfg->ADCBufData.dataSize;

    // ADC Data
    if(detectionStats->wasCodeDetected)
    {

        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nCODE: 1\t");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));


        snprintf(output_float,
                 sizeof(output_float),
                 "Prompt:\t%f\r\n",
                 detectionStats->promptValue);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_float,
                           strlen(output_float));
    }
    else
    {
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nCODE: 0\t");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));


        snprintf(output_float,
                 sizeof(output_float),
                 "Prompt:\t%f\r\n",
                 detectionStats->promptValue);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_float,
                           strlen(output_float));
    }
    if(num_received >= 100 && detectionStats->wasCodeDetected)
    {
        Task_sleep(1);         // 1 millisecond

        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nTimes--------\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "Total: %d\r\n",
                 rangingData->processingTime);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "MagADC: %d\r\n",
                 rangingData->magAdcTime);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "FFT: %d\r\n",
                 rangingData->fftTime);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "Vecmul: %d\r\n",
                 rangingData->vecmulTime);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "IFFT: %d\r\n",
                 rangingData->ifftTime);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "MagIFFT: %d\r\n",
                 rangingData->magIfftTime);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_float,
                 sizeof(output_float),
                 "Early: %f\r\n",
                 detectionStats->earlyValue);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_float,
                           strlen(output_float));

        snprintf(output_float,
                 sizeof(output_float),
                 "Prompt: %f\r\n",
                 detectionStats->promptValue);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_float,
                           strlen(output_float));

        snprintf(output_float,
                 sizeof(output_float),
                 "Late: %f\r\n",
                 detectionStats->lateValue);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_float,
                           strlen(output_float));

        snprintf(output_float,
                 sizeof(output_float),
                 "LSlope: %f\r\n",
                 detectionStats->leftSlope);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_float,
                           strlen(output_float));

        snprintf(output_float,
                 sizeof(output_float),
                 "LInterc: %f\r\n",
                 detectionStats->leftIntercept);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_float,
                           strlen(output_float));

        snprintf(output_float,
                 sizeof(output_float),
                 "RSlope: %f\r\n",
                 detectionStats->rightSlope);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_float,
                           strlen(output_float));

        snprintf(output_float,
                 sizeof(output_float),
                 "RInter: %f\r\n",
                 detectionStats->rightIntercept);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_float,
                           strlen(output_float));

        snprintf(output_data,
                 sizeof(output_data),
                 "Index: %d\r\n",
                 detectionStats->promptIndex);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "Detected: %d\r\n",
                 detectionStats->wasCodeDetected);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "Coarse: %d\r\n",
                 detectionStats->coarsePeakTimeOffsetCycles);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "Refined: %d\r\n",
                 detectionStats->RefinedPeakTimePicoseconds);

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nadc_data\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",%d,%d",
                     pBuffer[2*index],
                     pBuffer[2*index+1]);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            if(index%100 == 0)
            {
                Task_sleep(1);         // 1 millisecond
            }
        }

        // magnitudeData
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nmagnitude_data\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data + 2*gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",%d,%d",
                     pBuffer[2*index + 0],
                     pBuffer[2*index + 1]);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            if(index%100 == 0)
            {
                Task_sleep(1);         // 1 millisecond
            }
        }

        // fftOfMagnitude
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nfft_of_mag\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data + 4*gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",%d,%d",
                     pBuffer[2*index + 0],
                     pBuffer[2*index + 1]);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            if(index%100 == 0)
            {
                Task_sleep(1);         // 1 millisecond
            }
        }

        // vectorMultiplyOfFFtedData - int 32s
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nvector_multiply\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data + 6*gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",%d,%d",
                     ((cmplx32ImRe_t *)pBuffer)[index].imag,
                     ((cmplx32ImRe_t *)pBuffer)[index].real);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            if(index%100 == 0)
            {
                Task_sleep(1);         // 1 millisecond
            }
        }

        // IFFT of two vectors multiplied together - int 32s
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nifft_data\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data + 10*gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",%d,%d",
                     ((int32_t *)pBuffer)[2*index + 0],
                     ((int32_t *)pBuffer)[2*index + 1]);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            if(index%100 == 0)
            {
                Task_sleep(1);         // 1 millisecond
            }
        }

        // Magnitude of the IFFT of two vectors multiplied together - int 32, but only one value per sample
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\nmag_ifft\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data + 14*gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",0,%f",
                     ((float *)pBuffer)[index]);
                    //((uint32_t *)pBuffer)[index]);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            if(index%100 == 0)
            {
                Task_sleep(1);         // 1 millisecond
            }
        }

        // Complex conjugate of the FFT'd gold code
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\ncmplx_conj_gold\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));

        pBuffer = (int16_t *)result->radarCube.data + 16*gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples;
        for(index = 0; index < gMmwMssMCB.subFrameCfg[result->subFrameIdx].numAdcSamples; index++)
        {
            snprintf(output_data,
                     sizeof(output_data),
                     ",%d,%d",
                     pBuffer[2*index + 0],
                     pBuffer[2*index + 1]);

            UART_writePolling (uartHandle,
                               (uint8_t*)&output_data,
                               strlen(output_data));

            if(index%100 == 0)
            {
                Task_sleep(1);         // 1 millisecond
            }
        }

        // A final \r\n
        snprintf(output_data,
                 sizeof(output_data),
                 "\r\n");

        UART_writePolling (uartHandle,
                           (uint8_t*)&output_data,
                           strlen(output_data));
    }
    /*

    // Header:
    header.platform =  0xA6843;
    header.magicWord[0] = 0x0102;
    header.magicWord[1] = 0x0304;
    header.magicWord[2] = 0x0506;
    header.magicWord[3] = 0x0708;
    header.version =    MMWAVE_SDK_VERSION_BUILD |
                        (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                        (MMWAVE_SDK_VERSION_MINOR << 16) |
                        (MMWAVE_SDK_VERSION_MAJOR << 24);

    packetLen = sizeof(Ranging_output_message_header);
    if (pGuiMonSel->noiseProfile)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_NOISE_PROFILE;
        //tl[tlvIdx].length = sizeof(uint16_t) * subFrameCfg->numRangeBins;
        tl[tlvIdx].length = sizeof(uint16_t) * subFrameCfg->numAdcSamples;
        packetLen += sizeof(Ranging_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->statsInfo)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_STATS;
        tl[tlvIdx].length = sizeof(Ranging_output_message_stats);
        packetLen += sizeof(Ranging_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;

        Ranging_getTemperatureReport();
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS;
        tl[tlvIdx].length = sizeof(Ranging_temperatureStats);
        packetLen += sizeof(Ranging_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }

    header.numTLVs = tlvIdx;
    // Round up packet length to multiple of RANGING_OUTPUT_MSG_SEGMENT_LEN
    header.totalPacketLen = RANGING_OUTPUT_MSG_SEGMENT_LEN *
            ((packetLen + (RANGING_OUTPUT_MSG_SEGMENT_LEN-1))/RANGING_OUTPUT_MSG_SEGMENT_LEN);
    header.timeCpuCycles = Pmu_getCount(0);
    header.frameNumber = stats->frameStartIntCounter;
    header.subFrameNumber = result->subFrameIdx;

    UART_writePolling (uartHandle,
                       (uint8_t*)&header,
                       sizeof(Ranging_output_message_header));

    tlvIdx = 0;

    // Send noise profile
    if (pGuiMonSel->noiseProfile)
    {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(Ranging_output_message_tl));
        tlvIdx++;
    }

    // Send stats information
    if (pGuiMonSel->statsInfo == 1)
    {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(Ranging_output_message_tl));

        // Address translation is done when buffer is received
        UART_writePolling (uartHandle,
                           (uint8_t*)timingInfo,
                           tl[tlvIdx].length);
        tlvIdx++;
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(Ranging_output_message_tl));
        UART_writePolling (uartHandle,
                           (uint8_t*)&gMmwMssMCB.temperatureStats,
                           tl[tlvIdx].length);
        tlvIdx++;
    }

    // Send padding bytes
    numPaddingBytes = RANGING_OUTPUT_MSG_SEGMENT_LEN - (packetLen & (RANGING_OUTPUT_MSG_SEGMENT_LEN-1));
    if (numPaddingBytes<RANGING_OUTPUT_MSG_SEGMENT_LEN)
    {
        UART_writePolling (uartHandle,
                            (uint8_t*)padding,
                            numPaddingBytes);
    }
    */
    num_received++;
}


/**
 *  @b Description
 *  @n
 *      Help function to make DPM_ioctl blocking until response is reported
 *
 *  @retval
 *      Success         -0
 *      Failed          <0
 */
int32_t Ranging_DPM_ioctl_blocking
(
    DPM_Handle handle,
    uint32_t cmd,
    void* arg,
    uint32_t argLen
)
{
    int32_t retVal = 0;

    retVal = DPM_ioctl(handle,
                     cmd,
                     arg,
                     argLen);

    if(retVal == 0)
    {
        /* Wait until ioctl completed */
        Semaphore_pend(gMmwMssMCB.DPMioctlSemHandle, BIOS_WAIT_FOREVER);
    }

    return(retVal);
}


/**
 *  @b Description
 *  @n
 *      Utility function to get previous sub-frame index
 *
 *  @param[in] currentIndx      Current sub-frame index
 *  @param[in] numSubFrames     Number of sub-frames
 *
 *  @retval
 *      Index of previous sub-frame
 */
static uint8_t Ranging_getPrevSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames)
{
    uint8_t prevIndx;

    if (currentIndx == 0)
    {
        prevIndx = numSubFrames - 1;
    }
    else
    {
        prevIndx = currentIndx - 1;
    }
    return(prevIndx);
}


/**
 *  @b Description
 *  @n
 *      Utility function to get next sub-frame index
 *
 *  @param[in] currentIndx      Current sub-frame index
 *  @param[in] numSubFrames     Number of sub-frames
 *
 *  @retval
 *      Index of next sub-frame.
 */
static uint8_t Ranging_getNextSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames)
{
    uint8_t nextIndx;

    if (currentIndx == (numSubFrames - 1))
    {
        nextIndx = 0;
    }
    else
    {
        nextIndx = currentIndx + 1;
    }
    return(nextIndx);
}
