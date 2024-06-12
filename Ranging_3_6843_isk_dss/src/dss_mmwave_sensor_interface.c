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
