/*
 * mmwave_sensor_interface_dss.c
 *
 *  Created on: May 26, 2024
 *      Author: LeeLemay
 */

#include <stdint.h>
#include <inc/dss_mmwave_sensor_interface.h>
#include <xdc/runtime/System.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/alg/mmwavelib/mmwavelib.h>
#include <ti/control/mmwave/include/mmwave_internal.h>
#include <ti/sysbios/BIOS.h>
#include <inc/ranging_dss.h>


#pragma SET_CODE_SECTION(".l1pcode")

extern Ranging_DSS_MCB    gMmwDssMCB;

/**
 * @brief
 *  Global Variable for LDO BYPASS config, PLease consult your
 * board/EVM user guide before changing the values here
 */
rlRfLdoBypassCfg_t gRFLdoBypassCfg =
{
    .ldoBypassEnable   = 0, /* 1.0V RF supply 1 and 1.0V RF supply 2 */
    .supplyMonIrDrop   = 0, /* IR drop of 3% */
    .ioSupplyIndicator = 0, /* 3.3 V IO supply */
};

/* Calibration Data Save/Restore defines */
#define MMWDEMO_CALIB_FLASH_SIZE                  4096
#define MMWDEMO_CALIB_STORE_MAGIC            (0x7CB28DF9U)

Ranging_calibData gCalibDataStorage;
#pragma DATA_ALIGN(gCalibDataStorage, 8);

int32_t MMWave_start_part_one_internal (MMWave_Handle mmWaveHandle, const MMWave_CalibrationCfg* ptrCalibrationCfg, int32_t* errCode);
int32_t MMWave_start_part_two_internal (MMWave_Handle mmWaveHandle, const MMWave_CalibrationCfg* ptrCalibrationCfg, int32_t* errCode);
int32_t MMWave_start_internal (MMWave_Handle mmWaveHandle, const MMWave_CalibrationCfg* ptrCalibrationCfg, int32_t* errCode);

/*
int16_t startSensorPartOne()
{
    int32_t errCode;
    MMWave_CalibrationCfg calibrationCfg;

    // Initialize the calibration configuration:
    memset((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

    // Populate the calibration configuration:
    // calibrationCfg.dfeDataOutputMode                          = MMWave_DFEDataOutputMode_FRAME;
    calibrationCfg.dfeDataOutputMode =
        gMmwDssMCB.ctrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = false;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = false;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    // Start the mmWave module: The configuration has been applied successfully.
    if (MMWave_start_part_one_internal(gMmwDssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        // Error: Unable to start the mmWave control //
        System_printf("Error: MMWDemoDSS mmWave Start failed [Error code %d]\n", errCode);
        return -1;
    }
    return 0;
}

int16_t startSensorPartTwo()
{
    int32_t errCode;
    MMWave_CalibrationCfg calibrationCfg;

    // Initialize the calibration configuration:
    memset((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

    // Populate the calibration configuration:
    // calibrationCfg.dfeDataOutputMode                          = MMWave_DFEDataOutputMode_FRAME;
    calibrationCfg.dfeDataOutputMode =
        gMmwDssMCB.ctrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = false;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = false;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    // Start the mmWave module: The configuration has been applied successfully.
    if (MMWave_start_part_two_internal(gMmwDssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        // Error: Unable to start the mmWave control //
        System_printf("Error: MMWDemoDSS mmWave Start failed [Error code %d]\n", errCode);
        return -1;
    }
    gMmwDssMCB.sensorState = Ranging_SensorState_STARTED;
    return 0;
}
*/
int16_t startSensor()
{
    int32_t errCode;
    MMWave_CalibrationCfg calibrationCfg;

    // Initialize the calibration configuration:
    memset((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

    // Populate the calibration configuration:
    // calibrationCfg.dfeDataOutputMode                          = MMWave_DFEDataOutputMode_FRAME;
    calibrationCfg.dfeDataOutputMode = gMmwDssMCB.ctrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = false;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = false;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

#ifdef SUBSYS_DSS
    Semaphore_pend(gMmwDssMCB.sensorConfigSemaphore, BIOS_WAIT_FOREVER);
#endif

    // Start the mmWave module: The configuration has been applied successfully.
    if (MMWave_start_internal(gMmwDssMCB.ctrlHandle, &calibrationCfg, &errCode))
    {
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to do one time sensor initialization.
 *      User need to fill gMmwMssMCB.openCfg before calling this function
 *
 *  @param[in]  isFirstTimeOpen     If true then issues MMwave_open
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Ranging_openSensor(bool isFirstTimeOpen)
{
    int32_t             errCode;
    MMWave_ErrorLevel   errorLevel;
    int16_t             mmWaveErrorCode;
    int16_t             subsysErrorCode;
    int32_t             retVal;
    MMWave_CalibrationData     calibrationDataCfg;
    MMWave_CalibrationData     *ptrCalibrationDataCfg;

    /*  Open mmWave module, this is only done once */
    if (isFirstTimeOpen == true)
    {

        System_printf ("Debug: Sending rlRfSetLdoBypassConfig with %d %d %d\n",
                                            gRFLdoBypassCfg.ldoBypassEnable,
                                            gRFLdoBypassCfg.supplyMonIrDrop,
                                            gRFLdoBypassCfg.ioSupplyIndicator);
        retVal = rlRfSetLdoBypassConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlRfLdoBypassCfg_t*)&gRFLdoBypassCfg);
        if(retVal != 0)
        {
            System_printf("Error: rlRfSetLdoBypassConfig retVal=%d\n", retVal);
            return -1;
        }

        /*  Open mmWave module, this is only done once */
        /* Setup the calibration frequency */
        gMmwDssMCB.openCfg.freqLimitLow = 600U;
        gMmwDssMCB.openCfg.freqLimitHigh = 640U;

        /* start/stop async events */
        gMmwDssMCB.openCfg.disableFrameStartAsyncEvent = false;
        gMmwDssMCB.openCfg.disableFrameStopAsyncEvent  = false;

        /* No custom calibration: */
        gMmwDssMCB.openCfg.useCustomCalibration        = false;
        gMmwDssMCB.openCfg.customCalibrationEnableMask = 0x0;

        /* calibration monitoring base time unit
         * setting it to one frame duration as the demo doesnt support any
         * monitoring related functionality
         */
        gMmwDssMCB.openCfg.calibMonTimeUnit            = 1;

        if( (gMmwDssMCB.calibCfg.saveEnable != 0) &&
        (gMmwDssMCB.calibCfg.restoreEnable != 0) )
        {
            /* Error: only one can be enabled at at time */
            System_printf ("Error: Ranging failed with both save and restore enabled.\n");
            return -1;
        }

        if(gMmwDssMCB.calibCfg.restoreEnable != 0)
        {
            if(Ranging_calibRestore(&gCalibDataStorage) < 0)
            {
                System_printf ("Error: Ranging failed restoring calibration data from flash.\n");
                return -1;
            }

            /*  Boot calibration during restore: Disable calibration for:
                 - Rx gain,
                 - Rx IQMM,
                 - Tx phase shifer,
                 - Tx Power

                 The above calibration data will be restored from flash. Since they are calibrated in a control
                 way to avoid interference and spec violations.
                 In this demo, other bit fields(except the above) are enabled as indicated in customCalibrationEnableMask to perform boot time
                 calibration. The boot time calibration will overwrite the restored calibration data from flash.
                 However other bit fields can be disabled and calibration data can be restored from flash as well.

                 Note: In this demo, calibration masks are enabled for all bit fields when "saving" the data.
            */
            gMmwDssMCB.openCfg.useCustomCalibration        = true;
            gMmwDssMCB.openCfg.customCalibrationEnableMask = 0x1F0U;

            calibrationDataCfg.ptrCalibData = &gCalibDataStorage.calibData;
            calibrationDataCfg.ptrPhaseShiftCalibData = &gCalibDataStorage.phaseShiftCalibData;
            ptrCalibrationDataCfg = &calibrationDataCfg;
        }
        else
        {
            ptrCalibrationDataCfg = NULL;
        }

        /* Open the mmWave module: */
        if (MMWave_open (gMmwDssMCB.ctrlHandle, &gMmwDssMCB.openCfg, ptrCalibrationDataCfg, &errCode) < 0)
        {
            /* Error: decode and Report the error */
            MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
            System_printf ("Error: mmWave Open failed [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);
            return -1;
        }

        /* Save calibration data in flash */
        if(gMmwDssMCB.calibCfg.saveEnable != 0)
        {

            retVal = rlRfCalibDataStore(RL_DEVICE_MAP_INTERNAL_BSS, &gCalibDataStorage.calibData);
            if(retVal != RL_RET_CODE_OK)
            {
                /* Error: Calibration data restore failed */
             System_printf("MSS demo failed rlRfCalibDataStore with Error[%d]\n", retVal);
                return -1;
            }

#if (defined(SOC_XWR18XX) || defined(SOC_XWR68XX))

        /* update txIndex in all chunks to get data from all Tx.
           This should be done regardless of num TX channels enabled in MMWave_OpenCfg_t::chCfg or number of Tx
           application is interested in. Data for all existing Tx channels should be retrieved
           from RadarSS and in the order as shown below.
           RadarSS will return non-zero phase shift values for all the channels enabled via
           MMWave_OpenCfg_t::chCfg and zero phase shift values for channels disabled in MMWave_OpenCfg_t::chCfg */
            gCalibDataStorage.phaseShiftCalibData.PhShiftcalibChunk[0].txIndex = 0;
            gCalibDataStorage.phaseShiftCalibData.PhShiftcalibChunk[1].txIndex = 1;
            gCalibDataStorage.phaseShiftCalibData.PhShiftcalibChunk[2].txIndex = 2;

            /* Basic validation passed: Restore the phase shift calibration data */
            retVal = rlRfPhShiftCalibDataStore(RL_DEVICE_MAP_INTERNAL_BSS, &(gCalibDataStorage.phaseShiftCalibData));
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Phase shift Calibration data restore failed */
             System_printf("MSS demo failed rlRfPhShiftCalibDataStore with Error[%d]\n", retVal);
                return retVal;
            }
#endif
            /* Save data in flash */
            retVal = Ranging_calibSave(&gMmwDssMCB.calibCfg.calibDataHdr, &gCalibDataStorage);
            if(retVal < 0)
            {
                return retVal;
            }
        }

        /*Set up HSI clock*/
        if(Ranging_mssSetHsiClk() < 0)
        {
            System_printf ("Error: Ranging_mssSetHsiClk failed.\n");
            return -1;
        }

        /* Open the datapath modules that runs on MSS */
        //Ranging_dataPathOpen();
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the mmWave
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void Ranging_dssMMWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute(gMmwDssMCB.ctrlHandle, &errCode) < 0)
            System_printf("Error: MMWDemoDSS mmWave control execution failed [Error code %d]\n", errCode);
    }
}

/**
 *  @b Description
 *  @n
 *      Registered open callback function which is invoked when the mmWave module
 *      has been opened on the MSS
 *
 *  @param[in]  ptrOpenCfg
 *      Pointer to the open configuration
 *
 *  @retval
 *      Not applicable
 */
static void Ranging_dssMmwaveOpenCallbackFxn(MMWave_OpenCfg *ptrOpenCfg)
{
    // Save the configuration
    memcpy((void *)(&gMmwDssMCB.openCfg), (void *)ptrOpenCfg, sizeof(MMWave_OpenCfg));
    gMmwDssMCB.sensorState = Ranging_SensorState_OPENED;
    return;
}
/**
 *  @b Description
 *  @n
 *      Registered close callback function which is invoked when the mmWave module
 *      has been closed on the MSS
 *
 *  @retval
 *      Not applicable
 */
static void Ranging_dssMmwaveCloseCallbackFxn(void)
{
    //gMmwDssMCB.stats.closeEvt++;
    return;
}

/**
 *  @b Description
 *  @n
 *      Registered config callback function on DSS which is invoked by MMWAVE library when the remote side
 *  has finished configure mmWaveLink and BSS. The configuration need to be saved on DSS and used for DataPath.
 *
 *  @param[in]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
static void Ranging_dssMmwaveConfigCallbackFxn(MMWave_CtrlCfg *ptrCtrlCfg)
{
    // Save the configuration
    memcpy((void *)(&gMmwDssMCB.ctrlCfg), (void *)ptrCtrlCfg, sizeof(MMWave_CtrlCfg));
    Semaphore_post(gMmwDssMCB.sensorConfigSemaphore);
    return;
}

/**
 *  @b Description
 *  @n
 *      Registered Start callback function on DSS which is invoked by MMWAVE library
 *    when the remote side has started mmWaveLink and BSS. This Callback function passes
 *    the event to DataPath task.
 *
 *  @retval
 *      Not applicable
 */
static void Ranging_dssMmwaveStartCallbackFxn(MMWave_CalibrationCfg *ptrCalibrationCfg)
{
    gMmwDssMCB.sensorState = Ranging_SensorState_STARTED;
    gMmwDssMCB.stats.sensorStartCount++;
}

/**
 *  @b Description
 *  @n
 *      Registered Start callback function on DSS which is invoked by MMWAVE library
 *    when the remote side has stop mmWaveLink and BSS. This Callback function passes
 *    the event to DataPath task.
 *
 *  @retval
 *      Not applicable
 */
static void Ranging_dssMmwaveStopCallbackFxn(void)
{
    gMmwDssMCB.sensorState = Ranging_SensorState_STOPPED;
    gMmwDssMCB.stats.sensorStopCount++;
}

/**
 *  @b Description
 *  @n
 *      Registered event function to mmwave which is invoked when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always return 0
 */
static int32_t Ranging_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

    /* Process the received message: //
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            // Received Asychronous Message: //
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                {
                    rlCpuFault_t *rfCpuFault = (rlCpuFault_t *)payload;
                    System_printf("Debug: CPU Fault has been detected\n");
                    CLI_write("ERROR: Fault \n type: %d, lineNum: %d, LR: 0x%x \n"
                                "PrevLR: 0x%x, spsr: 0x%x, sp: 0x%x, PC: 0x%x \n"
                                "Status: 0x%x, Source: %d, AxiErrType: %d, AccType: %d, Recovery Type: %d \n",
                                rfCpuFault->faultType,
                                rfCpuFault->lineNum,
                                rfCpuFault->faultLR,
                                rfCpuFault->faultPrevLR,
                                rfCpuFault->faultSpsr,
                                rfCpuFault->faultSp,
                                rfCpuFault->faultAddr,
                                rfCpuFault->faultErrStatus,
                                rfCpuFault->faultErrSrc,
                                rfCpuFault->faultAxiErrType,
                                rfCpuFault->faultAccType,
                                rfCpuFault->faultRecovType);
                    Ranging_debugAssert(0);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    CLI_write("ERROR: ESM Fault. Group1:[0x%x] Group2:[0x%x]\n",
                                ((rlBssEsmFault_t *)payload)->esmGrp1Err,
                                ((rlBssEsmFault_t *)payload)->esmGrp2Err);
                    Ranging_debugAssert(0);
                    break;
                }
                case RL_RF_AE_ANALOG_FAULT_SB:
                {
                    Ranging_debugAssert(0);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    rlRfInitComplete_t*  ptrRFInitCompleteMessage;
                    uint32_t            calibrationStatus;

                    // Get the RF-Init completion message: //
                    ptrRFInitCompleteMessage = (rlRfInitComplete_t*)payload;
                    calibrationStatus = ptrRFInitCompleteMessage->calibStatus & 0x1FFFU;

                    // Display the calibration status: //
                    CLI_write ("Debug: Init Calibration Status = 0x%x\n", calibrationStatus);
                    break;
                }

                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    gMmwMssMCB.stats.frameTriggerReady++;
                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    System_printf("Debug: Monitoring FAIL Report received \n");
                    gMmwMssMCB.stats.failedTimingReports++;
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    gMmwMssMCB.stats.calibrationReports++;
                    break;
                }
                case RL_RF_AE_FRAME_END_SB:
                {
                    gMmwMssMCB.stats.sensorStopped++;
                    DebugP_log0("App: BSS stop (frame end) received\n");

                    Ranging_dataPathStop();
                    break;
                }

                default:
                {
                    System_printf ("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
                    break;
                }
            }
            break;
        }
        // Async Event from MMWL //
        case RL_MMWL_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
                case RL_MMWL_AE_MISMATCH_REPORT:
                {
                    // link reports protocol error in the async report from BSS //
                    Ranging_debugAssert(0);
                    break;
                }
                case RL_MMWL_AE_INTERNALERR_REPORT:
                {
                    // link reports internal error during BSS communication //
                    Ranging_debugAssert(0);
                    break;
                }
            }
            break;
        }
        default:
        {
            System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
            break;
        }
    }
    */
    return 0;
}


int32_t initializeMMWaveSystem()
{
    MMWave_InitCfg      initCfg;
    int32_t             errCode;

    /* Populate the init configuration for mmwave library: */
    initCfg.domain                      = MMWave_Domain_DSS;
    initCfg.socHandle                   = gMmwDssMCB.socHandle;
    initCfg.eventFxn                    = Ranging_eventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver     = 1U;
    initCfg.linkCRCCfg.crcChannel       = CRC_Channel_CH1;
    initCfg.cfgMode                     = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode               = MMWave_ExecutionMode_COOPERATIVE;
    initCfg.cooperativeModeCfg.cfgFxn   = Ranging_dssMmwaveConfigCallbackFxn;
    initCfg.cooperativeModeCfg.startFxn = Ranging_dssMmwaveStartCallbackFxn;
    initCfg.cooperativeModeCfg.stopFxn  = Ranging_dssMmwaveStopCallbackFxn;
    initCfg.cooperativeModeCfg.openFxn  = Ranging_dssMmwaveOpenCallbackFxn;
    initCfg.cooperativeModeCfg.closeFxn = Ranging_dssMmwaveCloseCallbackFxn;

    /* Initialize and setup the mmWave Control module */
    gMmwDssMCB.ctrlHandle = MMWave_init(&initCfg, &errCode);
    if (gMmwDssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        System_printf("Error: Ranging DSS mmWave Control Initialization failed [Error code %d]\n", errCode);
        return errCode;
    }
    System_printf("Debug: Ranging DSS mmWave Control Initialization succeeded\n");

    /******************************************************************************
     * TEST: Synchronization
     * - The synchronization API always needs to be invoked.
     ******************************************************************************/
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = MMWave_sync(gMmwDssMCB.ctrlHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the mmWave control module */
            System_printf("Error: MMWDemoDSS mmWave Control Synchronization failed [Error code %d]\n", errCode);
            return errCode;
        }
        if (syncStatus == 1)
        {
            /* Synchronization achieved: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }
    System_printf("Debug: Ranging DSS MMWave_sync succeeded\n");
    return 0;
}
