/**
 *   @file  mmw_mss.h
 *
 *   @brief
 *      This is the main header file for the Millimeter Wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef RANGING_MSS_H
#define RANGING_MSS_H

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

#include <ti/common/mmwave_error.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>

#include <inc/ranging_adcconfig.h>
#include <inc/ranging_monitor.h>
#include <inc/ranging_output.h>
//#include <ti/datapath/dpc/objectdetection/objdetrangehwa/objdetrangehwa.h>

#include <inc/ranging_config.h>
#include <inc/ranging_lvds_stream.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief For advanced frame config, below define means the configuration given is
 * global at frame level and therefore it is broadcast to all sub-frames.
 */
#define MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG (-1)

/*! @brief CFAR threshold encoding factor
 */
#define MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR (100.0)

/**
 * @defgroup configStoreOffsets     Offsets for storing CLI configuration
 * @brief    Offsets of config fields within the parent structures, note these offsets will be
 *           unique and hence can be used to differentiate the commands for processing purposes.
 * @{
 */
#define MMWDEMO_GUIMONSEL_OFFSET                 (offsetof(Ranging_SubFrameCfg, guiMonSel))
#define MMWDEMO_ADCBUFCFG_OFFSET                 (offsetof(Ranging_SubFrameCfg, adcBufCfg))
#define MMWDEMO_LVDSSTREAMCFG_OFFSET             (offsetof(Ranging_SubFrameCfg, lvdsStreamCfg))

#define MMWDEMO_BPMCFG_OFFSET                    (offsetof(Ranging_SubFrameCfg, bpmCfg))

#define MMWDEMO_SUBFRAME_DSPDYNCFG_OFFSET        (offsetof(Ranging_SubFrameCfg, objDetDynCfg) + \
                                                  offsetof(Ranging_DPC_ObjDet_DynCfg, dspDynCfg))

#define MMWDEMO_SUBFRAME_R4FDYNCFG_OFFSET        (offsetof(Ranging_SubFrameCfg, objDetDynCfg) + \
                                                  offsetof(Ranging_DPC_ObjDet_DynCfg, r4fDynCfg))

/** @}*/ /* configStoreOffsets */

/**
 * @brief
 *  Millimeter Wave Demo Sensor State
 *
 * @details
 *  The enumeration is used to define the sensor states used in mmwDemo
 */
typedef enum Ranging_SensorState_e
{
    /*!  @brief Inital state after sensor is initialized.
     */
    Ranging_SensorState_INIT = 0,

    /*!  @brief Inital state after sensor is post RF init.
     */
    Ranging_SensorState_OPENED,

    /*!  @brief Indicates sensor is started */
    Ranging_SensorState_STARTED,

    /*!  @brief  State after sensor has completely stopped */
    Ranging_SensorState_STOPPED
}Ranging_SensorState;

/**
 * @brief
 *  Millimeter Wave Demo statistics
 *
 * @details
 *  The structure is used to hold the statistics information for the
 *  Millimeter Wave demo
 */
typedef struct Ranging_MSS_Stats_t
{
    /*! @brief   Counter which tracks the number of frame trigger events from BSS */
    uint64_t     frameTriggerReady;
    
    /*! @brief   Counter which tracks the number of failed calibration reports
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     failedTimingReports;

    /*! @brief   Counter which tracks the number of calibration reports received
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     calibrationReports;

     /*! @brief   Counter which tracks the number of sensor stop events received
      *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     sensorStopped;

     /*! @brief   Counter which tracks the number of dpm stop events received
      *           The event is triggered by DPM_Report_DPC_STOPPED from DPM */
    uint32_t     dpmStopEvents;

    /*! @brief   Counter which tracks the number of dpm start events received
     *           The event is triggered by DPM_Report_DPC_STARTED from DPM */
    uint32_t     dpmStartEvents;

}Ranging_MSS_Stats;

/**
 * @brief
 *  The structure specifies the BPM configuration.
 *
 * @details
 *  The BPM is supported only for two azimuth transmit antennas.
 *  say A and B. In the even time slots (0,2,..), both transmit antennas should be configured 
 *  to transmit with positive phase i.e
 *  (A,B) = (+, +)
.*  In the odd time slots (1,3..), the transmit antennas should be configured to transmit with phase
 *  (A,B) = (+, -)
 *
 *  The BPM decoding will produce the virtual antenna array in the order A,B [not B,A] which will
 *  be used for AoA processing. So user must make sure that the A,B mapping to the physical
 *  transmit antennas corresponds to the intended virtual antenna order. On the 6843 EVM, this 
 *  means A = Tx1 and B = Tx3
 *
 *  Please note: mmwave link use TX antenna index starting from 0.
 */
typedef struct Ranging_BpmCfg_t
{
    /**
     * @brief   Enabled/disabled flag
     */
    bool        isEnabled;

    /**
     * @brief   
     *          If BPM is enabled, this is the chirp index for the first BPM chirp.
     *          It will have phase 0 on both Azimuth TX antennas (+ +).
     *
     *          If BPM is disabled, a BPM disable command (set phase to zero) will
     *          be issued for the chirps in the range [chirp0Idx..chirp1Idx].
     */
    uint16_t    chirp0Idx;

    /**
     * @brief   
     *          If BPM is enabled, this is the chirp index for the second BPM chirp.
     *          It will have phase 0 on the first Azimuth TX antenna and phase 180 
     *          on second Azimuth TX antenna (+ -).
     *
     *          If BPM is disabled, a BPM disable command (set phase to zero) will
     *          be issued for the chirps in the range [chirp0Idx..chirp1Idx].
     */
    uint16_t    chirp1Idx;
}Ranging_BpmCfg;

/**
 * @brief
 *  Millimeter Wave Demo Data Path Information.
 *
 * @details
 *  The structure is used to hold all the relevant information for
 *  the data path.
 */
typedef struct Ranging_SubFrameCfg_t
{
    /*! @brief ADC buffer configuration storage */
    Ranging_ADCBufCfg adcBufCfg;

    /*! @brief Flag indicating if @ref adcBufCfg is pending processing. */
    uint8_t isAdcBufCfgPending : 1;

    Ranging_BpmCfg      bpmCfg;

    /*! @brief Flag indicating if @ref bpmCfg is pending processing. */
    uint8_t isBpmCfgPending : 1;

    /*! @brief  LVDS stream configuration */
    Ranging_LvdsStreamCfg lvdsStreamCfg;

    /*! @brief Flag indicating if @ref lvdsStreamCfg is pending processing. */
    uint8_t isLvdsStreamCfgPending : 1;

    /*! @brief GUI Monitor selection configuration storage from CLI */
    Ranging_GuiMonSel guiMonSel;

    /*! @brief  ADCBUF will generate chirp interrupt event every this many chirps - chirpthreshold */
    uint8_t     numChirpsPerChirpEvent;

    /*! @brief  Number of bytes per RX channel, it is aligned to 16 bytes as required by ADCBuf driver  */
    uint32_t    adcBufChanDataSize;

    /*! @brief CQ signal & image band monitor buffer size */
    uint32_t    sigImgMonTotalSize;

    /*! @brief CQ RX Saturation monitor buffer size */
    uint32_t    satMonTotalSize;

    /*! @brief  Number of ADC samples */
    uint16_t    numAdcSamples;

    /*! @brief  Number of chirps per sub-frame */
    uint16_t    numChirpsPerSubFrame;
    
    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas; 
} Ranging_SubFrameCfg;

/*!
 * @brief
 * Structure holds message stats information from data path.
 *
 * @details
 *  The structure holds stats information. This is a payload of the TLV message item
 *  that holds stats information.
 */
typedef struct Ranging_SubFrameStats_t
{
    /*! @brief   Frame processing stats */
    Ranging_output_message_stats    outputStats;

    /*! @brief   Dynamic CLI configuration time in usec */
    uint32_t                        pendingConfigProcTime;

    /*! @brief   SubFrame Preparation time on MSS in usec */
    uint32_t                        subFramePreparationTime;
} Ranging_SubFrameStats;

/**
 * @brief Task handles storage structure
 */
typedef struct Ranging_TaskHandles_t
{
    /*! @brief   MMWAVE Control Task Handle */
    Task_Handle mmwaveCtrl;

    /*! @brief   ObjectDetection DPC related dpmTask */
    Task_Handle objDetDpmTask;

    /*! @brief   Demo init task */
    Task_Handle initTask;

    /*! @brief   Task to run the state machine */
    Task_Handle stateMachineTask;
} Ranging_taskHandles;

/*!
 * @brief
 * Structure holds temperature information from Radar front end.
 *
 * @details
 *  The structure holds temperature stats information. 
 */
typedef struct Ranging_temperatureStats_t
{

    /*! @brief   retVal from API rlRfTempData_t - can be used to know 
                 if values in temperatureReport are valid */
    int32_t        tempReportValid;

    /*! @brief   detailed temperature report - snapshot taken just 
                 before shipping data over UART */
    rlRfTempData_t temperatureReport;

} Ranging_temperatureStats;

/*!
 * @brief
 * Structure holds calibration save configuration used during sensor open.
 *
 * @details
 *  The structure holds calibration save configuration.
 */
typedef struct Ranging_calibDataHeader_t
{
    /*! @brief      Magic word for calibration data header */
    uint32_t 	magic;

    /*! @brief      Header length */
    uint32_t 	hdrLen;

    /*! @brief      mmwLink version */
    rlSwVersionParam_t 	linkVer;

    /*! @brief      RadarSS version */
    rlFwVersionParam_t 	radarSSVer;

    /*! @brief      Data length */
    uint32_t 	dataLen;

    /*! @brief      data padding to make sure calib data is 8 bytes aligned */
    uint32_t      padding;
} Ranging_calibDataHeader;

/*!
 * @brief
 * Structure holds calibration save configuration used during sensor open.
 *
 * @details
 *  The structure holds calibration save configuration.
 */
typedef struct Ranging_calibCfg_t
{
    /*! @brief      Calibration data header for validation read from flash */
    Ranging_calibDataHeader    calibDataHdr;

    /*! @brief      Size of Calibraton data size includng header */
    uint32_t 		sizeOfCalibDataStorage;

    /*! @brief      Enable/Disable calibration save process  */
    uint32_t 		saveEnable;

    /*! @brief      Enable/Disable calibration restore process  */
    uint32_t 		restoreEnable;
	
    /*! @brief      Flash Offset to restore the data from */
    uint32_t 		flashOffset;
} Ranging_calibCfg;


/*!
 * @brief
 * Structure holds calibration restore configuration used during sensor open.
 *
 * @details
 *  The structure holds calibration restore configuration.
 */
typedef struct Ranging_calibData_t
{
    /*! @brief      Calibration data header for validation read from flash */
    Ranging_calibDataHeader    calibDataHdr;

    /*! @brief      Calibration data */
    rlCalibrationData_t               calibData;

    /*! @brief      Phase shift Calibration data */
    rlPhShiftCalibrationData_t     phaseShiftCalibData;

    /* Future: If more fields are added to this structure or RL definitions
        are changed, please add dummy padding bytes here if size of
        Ranging_calibData is not multiple of 8 bytes. */
} Ranging_calibData;

/**
 * @brief
 *  Millimeter Wave Demo MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Millimeter Wave demo
 */
typedef struct Ranging_MSS_MCB_t
{
    /*! @brief      Configuration which is used to execute the demo */
    Ranging_Cfg                 cfg;

    /*! * @brief    Handle to the SOC Module */
    SOC_Handle                  socHandle;

    /*! @brief      UART Logging Handle */
    UART_Handle                 loggingUartHandle;

    /*! @brief      UART Command Rx/Tx Handle */
    UART_Handle                 commandUartHandle;

    /*! @brief      This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*! @brief      ADCBuf driver handle */
    ADCBuf_Handle               adcBufHandle;

    /*! @brief   Handle of the EDMA driver, used for CBUFF */
    EDMA_Handle                 edmaHandle;

    /*! @brief   Number of EDMA event Queues (tc) */
    uint8_t                     numEdmaEventQueues;

    /*! @brief   True if need to poll for edma error (because error interrupt is not
     *           connected to the CPU on the device */
    bool                        isPollEdmaError;

    /*! @brief   True if need to poll for edma transfer controller error
     *           because at least one of the transfer controller error interrupts
     *           are not connected to the CPU on the device */
    bool                        isPollEdmaTransferControllerError;

    /*! @brief   EDMA error Information when there are errors like missing events */
    EDMA_errorInfo_t            EDMA_errorInfo;

    /*! @brief EDMA transfer controller error information. */
    EDMA_transferControllerErrorInfo_t EDMA_transferControllerErrorInfo;

    /*! @brief      DSP chain DPM Handle */
    DPM_Handle                  objDetDpmHandle;

    /*! @brief      Object Detection DPC common configuration */
    Ranging_DPC_ObjDet_CommonCfg objDetCommonCfg;

    /*! @brief      Object Detection DPC subFrame configuration */
    Ranging_SubFrameCfg         subFrameCfg[RL_MAX_SUBFRAMES];

    /*! @brief      sub-frame stats */
    Ranging_SubFrameStats       subFrameStats[RL_MAX_SUBFRAMES];

    /*! @brief      Demo Stats */
    Ranging_MSS_Stats           stats;

    /*! @brief      Task handle storage */
    Ranging_taskHandles         taskHandles;

    /*! @brief   RF frequency scale factor, = 2.7 for 60GHz device, = 3.6 for 76GHz device */
    double                      rfFreqScaleFactor;

    /*! @brief   Semaphore handle to signal DPM started from DPM report function */
    Semaphore_Handle            DPMstartSemHandle;

    /*! @brief   Semaphore handle to signal DPM stopped from DPM report function. */
    Semaphore_Handle            DPMstopSemHandle;

    /*! @brief   Semaphore handle to signal DPM ioctl from DPM report function. */
    Semaphore_Handle            DPMioctlSemHandle;

    /*! @brief    Sensor state */
    Ranging_SensorState         sensorState;

    /*! @brief   Tracks the number of sensor start */
    uint32_t                    sensorStartCount;

    /*! @brief   Tracks the number of sensor sop */
    uint32_t                    sensorStopCount;

    /*! @brief   CQ monitor configuration - Signal Image band data */
    rlSigImgMonConf_t           cqSigImgMonCfg[RL_MAX_PROFILES_CNT];

    /*! @brief   CQ monitor configuration - Saturation data */
    rlRxSatMonConf_t            cqSatMonCfg[RL_MAX_PROFILES_CNT];

    /*! @brief   Analog monitor bit mask */
    Ranging_AnaMonitorCfg       anaMonCfg;

    /*! @brief   this structure is used to hold all the relevant information
         for the mmw demo LVDS stream*/
    Ranging_LVDSStream_MCB_t    lvdsStream;

    /*! @brief   this structure is used to hold all the relevant information
     for the temperature report*/
    Ranging_temperatureStats    temperatureStats;

    /*! @brief   Calibration cofiguration for save/restore */
    Ranging_calibCfg                calibCfg;

    /*! @brief Flag indicating if @ref anaMonCfg is pending processing. */
    uint8_t isAnaMonCfgPending : 1;

    /*! @brief Flag indicating if @ref calibCfg is pending processing. */
    uint8_t isCalibCfgPending : 1;
} Ranging_MSS_MCB;

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

/* Functions to handle the actions need to move the sensor state */
extern int32_t Ranging_openSensor(bool isFirstTimeOpen);
extern int32_t Ranging_configSensor(void);
extern int32_t Ranging_startSensor(void);
extern void Ranging_stopSensor(void);

/* functions to manage the dynamic configuration */
extern void Ranging_CfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum);

/* Debug Functions */
extern void _Ranging_debugAssert(int32_t expression, const char *file, int32_t line);
#define Ranging_debugAssert(expression) {                                      \
                                         _Ranging_debugAssert(expression,      \
                                                  __FILE__, __LINE__);         \
                                         DebugP_assert(expression);             \
                                        }

#ifdef __cplusplus
}
#endif

#endif /* MMW_MSS_H */

