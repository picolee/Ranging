/*
 *   @file  mmw_cli.c
 *
 *   @brief
 *      Mmw (Milli-meter wave) DEMO CLI Implementation
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* BIOS/XDC Include Files. */
#include <xdc/runtime/System.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/uart/UART.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/mathutils/mathutils.h>

/* Demo Include Files */
#include <inc/ranging_config.h>
#include <inc/ranging_mss.h>
#include <inc/ranging_adcconfig.h>
#include <inc/ranging_rfparser.h>
#include <inc/gold_code.h>

#ifdef SYS_COMMON_XWR68XX_LOW_POWER_MODE_EN
/* Low Power Library Functions*/
#include <ti/utils/libsleep/libsleep_xwr68xx.h>
#endif

/**************************************************************************
 *************************** Local function prototype****************************
 **************************************************************************/

/* CLI Extended Command Functions */
static int32_t Ranging_CLISensorStart (int32_t argc, char* argv[]);
static int32_t Ranging_CLISensorStop (int32_t argc, char* argv[]);
static int32_t Ranging_CLIGuiMonSel (int32_t argc, char* argv[]);
static int32_t Ranging_CLIADCBufCfg (int32_t argc, char* argv[]);
static int32_t Ranging_CLIBpmCfg (int32_t argc, char* argv[]);
static int32_t Ranging_CLIChirpQualityRxSatMonCfg (int32_t argc, char* argv[]);
static int32_t Ranging_CLIChirpQualitySigImgMonCfg (int32_t argc, char* argv[]);
static int32_t Ranging_CLIAnalogMonitorCfg (int32_t argc, char* argv[]);
static int32_t Ranging_CLILvdsStreamCfg (int32_t argc, char* argv[]);
static int32_t Ranging_CLIConfigDataPort (int32_t argc, char* argv[]);
static int32_t Ranging_CLICalibDataSaveRestore(int32_t argc, char* argv[]);

#ifdef SYS_COMMON_XWR68XX_LOW_POWER_MODE_EN
static int32_t Ranging_CLIIdleModeCycle (int32_t argc, char* argv[]);
static int32_t Ranging_CLIIdleModeDown (int32_t argc, char* argv[]);
#endif
/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/

extern Ranging_MSS_MCB    gMmwMssMCB;

#ifdef SYS_COMMON_XWR68XX_LOW_POWER_MODE_EN
extern void idle_power_down(IdleModeCfg   idleModeCfg);
extern void idle_power_cycle(IdleModeCfg   idleModeCfg);
#endif

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/
#define MMWDEMO_DATAUART_MAX_BAUDRATE_SUPPORTED 3125000
#define RECEIVE_PROFILE_NUMBER 0
#define TRANSMIT_PROFILE_NUMBER 1

/**************************************************************************
 *************************** CLI  Function Definitions **************************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor start command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLISensorStart (int32_t argc, char* argv[])
{
    bool        doReconfig = true;
    int32_t     retVal = 0;

    /*  Only following command syntax will be supported 
        sensorStart
        sensorStart 0
    */
    if (argc == 2)
    {
        doReconfig = (bool) atoi (argv[1]);

        if (doReconfig == true)
        {
            CLI_write ("Error: Reconfig is not supported, only argument of 0 is\n"
                       "(do not reconfig, just re-start the sensor) valid\n");
            return -1;
        }
    }
    else
    {
        /* In case there is no argument for sensorStart, always do reconfig */
        doReconfig = true;
    }

    /***********************************************************************************
     * Do sensor state management to influence the sensor actions
     ***********************************************************************************/

    /* Error checking initial state: no partial config is allowed 
       until the first sucessful sensor start state */
    if ((gMmwMssMCB.sensorState == Ranging_SensorState_INIT) ||
         (gMmwMssMCB.sensorState == Ranging_SensorState_OPENED))
    {
        MMWave_CtrlCfg ctrlCfg;

        /* need to get number of sub-frames so that next function to check
         * pending state can work */
        CLI_getMMWaveExtensionConfig (&ctrlCfg);
        gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames =
            Ranging_RFParser_getNumSubFrames(&ctrlCfg);

        if (Ranging_isAllCfgInPendingState() == 0)
        {
            CLI_write ("Error: Full configuration must be provided before sensor can be started "
                       "the first time\n");

            /* Although not strictly needed, bring back to the initial value since we
             * are rejecting this first time configuration, prevents misleading debug. */
            gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames = 0;

            return -1;
        }
    }

    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        CLI_write ("Ignored: Sensor is already started\n");
        return 0;
    }

    if (doReconfig == false)
    {
         /* User intends to issue sensor start without config, check if no
            config was issued after stop and generate error if this is the case. */
         if (Ranging_isAllCfgInNonPendingState() == 0)
         {
             /* Message user differently if all config was issued or partial config was
                issued. */
             if (Ranging_isAllCfgInPendingState())
             {
                 CLI_write ("Error: You have provided complete new configuration, "
                            "issue \"sensorStart\" (without argument) if you want it to "
                            "take effect\n");
             }
             else
             {
                 CLI_write ("Error: You have provided partial configuration between stop and this "
                            "command and partial configuration cannot be undone."
                            "Issue the full configuration and do \"sensorStart\" \n");
             }
             return -1;
         }
    }
    else
    {
        /* User intends to issue sensor start with full config, check if all config
           was issued after stop and generate error if  is the case. */
        MMWave_CtrlCfg ctrlCfg;

        /* need to get number of sub-frames so that next function to check
         * pending state can work */
        CLI_getMMWaveExtensionConfig (&ctrlCfg);
        gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames =
            Ranging_RFParser_getNumSubFrames(&ctrlCfg);
        
        if (Ranging_isAllCfgInPendingState() == 0)
        {
            /* Message user differently if no config was issued or partial config was
               issued. */
            if (Ranging_isAllCfgInNonPendingState())
            {
                CLI_write ("Error: You have provided no configuration, "
                           "issue \"sensorStart 0\" OR provide "
                           "full configuration and issue \"sensorStart\"\n");
            }
            else
            {
                CLI_write ("Error: You have provided partial configuration between stop and this "
                           "command and partial configuration cannot be undone."
                           "Issue the full configuration and do \"sensorStart\" \n");
            }
            /* Although not strictly needed, bring back to the initial value since we
             * are rejecting this first time configuration, prevents misleading debug. */
            gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames = 0;
            return -1;
        }
    }

    /***********************************************************************************
     * Retrieve and check mmwave Open related config before calling openSensor
     ***********************************************************************************/

    /*  Fill demo's MCB mmWave openCfg structure from the CLI configs*/
    if (gMmwMssMCB.sensorState == Ranging_SensorState_INIT)
    {
        /* Get the open configuration: */
        CLI_getMMWaveExtensionOpenConfig (&gMmwMssMCB.cfg.openCfg);
        /* call sensor open */
        retVal = Ranging_openSensor(true);
        if(retVal != 0)
        {
            return -1;
        }
        gMmwMssMCB.sensorState = Ranging_SensorState_OPENED;
    }
    else
    {
        /* openCfg related configurations like chCfg, lowPowerMode, adcCfg
         * are only used on the first sensor start. If they are different
         * on a subsequent sensor start, then generate a fatal error
         * so the user does not think that the new (changed) configuration
         * takes effect, the board needs to be reboot for the new
         * configuration to be applied.
         */
        MMWave_OpenCfg openCfg;
        CLI_getMMWaveExtensionOpenConfig (&openCfg);
        /* Compare openCfg->chCfg*/
        if(memcmp((void *)&gMmwMssMCB.cfg.openCfg.chCfg, (void *)&openCfg.chCfg,
                          sizeof(rlChanCfg_t)) != 0)
        {
            Ranging_debugAssert(0);
        }
        
        /* Compare openCfg->lowPowerMode*/
        if(memcmp((void *)&gMmwMssMCB.cfg.openCfg.lowPowerMode, (void *)&openCfg.lowPowerMode,
                          sizeof(rlLowPowerModeCfg_t)) != 0)
        {
            Ranging_debugAssert(0);
        }
        /* Compare openCfg->adcOutCfg*/
        if(memcmp((void *)&gMmwMssMCB.cfg.openCfg.adcOutCfg, (void *)&openCfg.adcOutCfg,
                          sizeof(rlAdcOutCfg_t)) != 0)
        {
            Ranging_debugAssert(0);
        }
    }

    

    /***********************************************************************************
     * Retrieve mmwave Control related config before calling startSensor
     ***********************************************************************************/
    /* Get the mmWave ctrlCfg from the CLI mmWave Extension */
    if(doReconfig)
    {
        /* if Ranging_openSensor has non-first time related processing, call here again*/
        /* call sensor config */
        CLI_getMMWaveExtensionConfig (&gMmwMssMCB.cfg.ctrlCfg);
        retVal = Ranging_configSensor();
        if(retVal != 0)
        {
            return -1;
        }
    }
    retVal = Ranging_startSensor();
    if(retVal != 0)
    {
        return -1;
    }

    /***********************************************************************************
     * Set the state
     ***********************************************************************************/
    gMmwMssMCB.sensorState = Ranging_SensorState_STARTED;
    return 0;
}

int32_t Ranging_SetBaseConfiguration(float frequencyInGhz)
{
    // Set up a static argv large enough for the maximum expected arguments
    char* argv[15];
    int32_t argc;
    int32_t                 errCode;

    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // flushCfg
    //  CLI_MMWaveFlushCfg(argc, argv );

    /* Flush the configuration in the MMWave */
    if (MMWave_flushCfg (gMmwMssMCB.ctrlHandle, &errCode) < 0)
    {
        /* Error: Flushing the configuration failed. Return the error code back to the callee */
        return errCode;
    }

    /* Reset the global configuration: */
    memset ((void*)&gMmwMssMCB.cfg.ctrlCfg, 0, sizeof(MMWave_CtrlCfg));

    /* Reset the open configuration: */
    memset ((void*)&gMmwMssMCB.cfg.openCfg, 0, sizeof(MMWave_OpenCfg));

    ////////////////////////////////////////////////////////////////////////////
    //  CLI_MMWaveDataOutputMode
    // Options are frame, continuous, and advanced
    // argc = 2;
    // argv[0] = "dfeDataOutputMode";
    // argv[1] = "1";                          // MMWave_DFEDataOutputMode_FRAME
    // CLI_MMWaveDataOutputMode(argc, argv);
    gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_FRAME;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Antenna Configuration
    //<rxChannelEn>          4       b0 RX0 Channel Enable, b1 RX1 Channel Enable, b2 RX2 Channel Enable, b3 RX3 Channel Enable, b15:4 - RESERVED
    //<txChannelEn>          1       b0 TX0 Channel Enable, b1 TX1 Channel Enable, b2 TX2 Channel Enable, b15:3 - RESERVED \n
    //<cascading>            0       0x0000 SINGLECHIP, 0x0001 MULTICHIP_MASTER, 0x0002 MULTICHIP_SLAVE

    /* Initialize the channel configuration: */
    memset ((void *)&gMmwMssMCB.cfg.openCfg.chCfg, 0, sizeof(rlChanCfg_t));
    gMmwMssMCB.cfg.openCfg.chCfg.rxChannelEn = 4;  // RX2 Channel Enable
    gMmwMssMCB.cfg.openCfg.chCfg.txChannelEn = 1;  // TX0 Channel Enable (however, we never transmit in RX mode.)
    gMmwMssMCB.cfg.openCfg.chCfg.cascading   = 0;  // cascading is off

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ADC Config           adcCfg 2 1
    //<numADCBits>           2       ADC out bits - 0(12 Bits), 1(14 Bits), 2(16 Bits)
    //<adcOutputFmt>         1       ADC out format- 0(Real), 1(Complex), 2(Complex with Image band), 3(Pseudo Real)
    memset ((void *)&gMmwMssMCB.cfg.openCfg.adcOutCfg, 0, sizeof(rlAdcOutCfg_t));
    gMmwMssMCB.cfg.openCfg.adcOutCfg.fmt.b2AdcBits   = 2;              // 16 output bits
    gMmwMssMCB.cfg.openCfg.adcOutCfg.fmt.b2AdcOutFmt = 1;              // Complex output

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ADC Config           adcbufCfg -1 0 1 1 1
    //<subFrameIdx>          -1
    //<adcOutputFmt>         0       0-Complex, 1-Real
    //<SampleSwap>           1       0-I in LSB, Q in MSB, 1-Q in LSB, I in MSB
    //<ChanInterleave>       1       0-interleaved(not supported on XWR16xx), 1- non-interleaved
    //<ChirpThreshold>       1       Chirp Threshold configuration used for ADCBUF buffer
    argc = 3;
    argv[0] = "adcbufCfg";
    argv[1] = "-1";             // subFrameIndex
    argv[2] = "0";              // Real
    argv[3] = "1";              // Q in LSB, I in MSB
    argv[4] = "1";              // non-interleaved
    argv[5] = "1";              // Chirp Threshold configuration used for ADCBUF buffer
    Ranging_CLIADCBufCfg(argc, argv);

    //////////////////////////////////////////////////////////////////////////////////////////
    // Bit Phase Modulation     bpmCfg -1 0 0 0
    //<chirpStartIdx>        0       % Chirp Start Index, Valid Range 0 -511
    //<chirpEndIdx>          0       % Chirp End Index, Valid Range chirpStartIdx -511
    //<constBpmVal>          0       % 0 for zero phase, 1 for 180 phase
    //bpmCfgAdvanced
    argc = 5;
    argv[0] = "bpmCfg";
    argv[1] = "-1";          // chirpStartIdx
    argv[2] = "0";          // chirpEndIdx
    argv[3] = "0";          // numLoops
    argv[4] = "0";          // numFrames
    Ranging_CLIBpmCfg(argc, argv);

    //////////////////////////////////////////////////////////////////////////////////////////
    // Low Power Configuration     lowPower 0 0

    /* Initialize the channel configuration: */
    memset ((void *)&gMmwMssMCB.cfg.openCfg.lowPowerMode, 0, sizeof(rlLowPowerModeCfg_t));

    /* Populate the channel configuration: */
    gMmwMssMCB.cfg.openCfg.lowPowerMode.lpAdcMode     = atoi (argv[2]);

    //////////////////////////////////////////////////////////////////////////////////////////
    // GUI Monitor     guiMonitor -1 1 1 0 0 0 1
    //guiMonSel.detectedObjects           = atoi (argv[2]);
    //guiMonSel.logMagRange               = atoi (argv[3]);
    //guiMonSel.noiseProfile              = atoi (argv[4]);
    //guiMonSel.rangeAzimuthHeatMap       = atoi (argv[5]);
    //guiMonSel.rangeDopplerHeatMap       = atoi (argv[6]);
    //guiMonSel.statsInfo                 = atoi (argv[7]);
    argc = 8;
    argv[0] = "guiMonitor";
    argv[1] = "-1";         // frameId
    argv[2] = "1";          // detectedObjects
    argv[3] = "1";          // logMagRange
    argv[4] = "0";          // noiseProfile
    argv[5] = "0";          // rangeAzimuthHeatMap
    argv[6] = "0";          // rangeDopplerHeatMap
    argv[7] = "1";          // statsInfo
    Ranging_CLIGuiMonSel(argc, argv);

    ////////////////////////////////////////////////////////////////////////////////////////
    // LVDS config    lvdsStreamCfg -1 0 1 0
    //<subFrameIdx>
    //<enableHeader>
    //<dataFmt>
    //<enableSW>
    argc = 5;
    argv[0] = "lvdsStreamCfg";
    argv[1] = "-1";         // subFrameIdx
    argv[2] = "0";          // enableHeader
    argv[3] = "1";          // dataFmt
    argv[4] = "0";          // enableSW
    Ranging_CLILvdsStreamCfg(argc, argv);

    ////////////////////////////////////////////////////////////////////////////////////////
    // Chirp Quality Saturation Monitor    CQRxSatMonitor 0 3 5 121 0
    //<profile>
    //<satMonSel>
    //<priSliceDuration>
    //<numSlices>
    //<rxChanMask>
    argc = 6;
    argv[0] = "CQRxSatMonitor";
    argv[1] = "0";          // profile
    argv[2] = "3";          // satMonSel
    argv[3] = "5";          // priSliceDuration
    argv[4] = "121";        // numSlices
    argv[5] = "0";          // rxChanMask
    Ranging_CLIChirpQualityRxSatMonCfg(argc, argv);

    ////////////////////////////////////////////////////////////////////////////////////////
    // Chirp Quality SigImg Monitor   CQSigImgMonitor 0 127 4
    //<profile>
    //<numSlices>
    //<numSamplePerSlice>
    argc = 4;
    argv[0] = "CQSigImgMonitor";
    argv[1] = "0";          // profile
    argv[2] = "127";        // numSlices
    argv[3] = "4";          // numSamplePerSlice
    Ranging_CLIChirpQualitySigImgMonCfg(argc, argv);

    ////////////////////////////////////////////////////////////////////////////////////////
    // Analog Monitor   analogMonitor 0 0
    //<rxSaturation>
    //<sigImgBand>
    argc = 3;
    argv[0] = "analogMonitor";
    argv[1] = "0";          // rxSaturation
    argv[2] = "0";          // sigImgBand
    Ranging_CLIAnalogMonitorCfg(argc, argv);


    ////////////////////////////////////////////////////////////////////////////////////////
    // Calibration Data   calibData 0 0 0
    //<save enable>
    //<restore enable>
    //<Flash offset>
    argc = 4;
    argv[0] = "calibData";
    argv[1] = "0";          // save enable
    argv[2] = "0";          // restore enable
    argv[3] = "0";          // restore enable
    Ranging_CLICalibDataSaveRestore(argc, argv);

    return 0;
}

int32_t Ranging_SetReceiveConfiguration(float frequencyInGhz)
{
    rlProfileCfg_t          profileCfg;
    int32_t                 errCode;
    MMWave_ProfileHandle    profileHandle;
    MMWave_ProfileHandle*   ptrBaseCfgProfileHandle;
    rlChirpCfg_t            chirpCfg;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Profile Config Zero          profileCfg 0 63.95 3 5 1280 0 0 0 0 4096 4000 0 0 158

    /* Initialize the profile configuration: */
    memset ((void *)&profileCfg, 0, sizeof(rlProfileCfg_t));
    if (gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode == MMWave_DFEDataOutputMode_FRAME)
    {
        ptrBaseCfgProfileHandle = &gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.profileHandle[0U];
    }
    else
    {
        ptrBaseCfgProfileHandle = &gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.profileHandle[0U];
    }

    if(ptrBaseCfgProfileHandle[RECEIVE_PROFILE_NUMBER] != NULL)
    {
        System_printf("Error: Receive profile already set.\n");
        Ranging_debugAssert (0);
    }

    /* Populate the profile configuration: */
    profileCfg.profileId             = RECEIVE_PROFILE_NUMBER;

    /* Translate from GHz to [1 LSB = gCLI_mmwave_freq_scale_factor * 1e9 / 2^26 Hz] units
     * of mmwavelink format */
    profileCfg.startFreqConst        = (uint32_t) (frequencyInGhz * (1U << 26) / gMmwMssMCB.rfFreqScaleFactor);

    /* Translate below times from us to [1 LSB = 10 ns] units of mmwavelink format */
    profileCfg.idleTimeConst         = (uint32_t)((float)3 * 1000 / 10);    // 3 us idle time
    profileCfg.adcStartTimeConst     = (uint32_t)((float)5 * 1000 / 10);    // 5 us ADC wait time
    profileCfg.rampEndTime           = (uint32_t)((float)1280 * 1000 / 10); // 1280 us ramp time
    profileCfg.txOutPowerBackoffCode = 0;                                   // 0 dB Tx decrease
    profileCfg.txPhaseShifter        = 0;                                   // no phase shift
    profileCfg.freqSlopeConst        = 0;                                   // No slope
    profileCfg.txStartTime           = (int32_t)((float)0 * 1000 / 10);     // 10's of ns
    profileCfg.numAdcSamples         = 4096;                                // 1024 us @ 4 MSPS
    profileCfg.digOutSampleRate      = 4000;                                // KSPS
    profileCfg.hpfCornerFreq1        = 0;                                   // 175 kHz - hpfCornerFreq1
    profileCfg.hpfCornerFreq2        = 0;                                   // 350 kHz - hpfCornerFreq2
    profileCfg.rxGain                = 158;                                 // 36 dB Rx Gain

    /* Add the profile to the mmWave module: */
    profileHandle = MMWave_addProfile (gMmwMssMCB.ctrlHandle, &profileCfg, &errCode);
    if (profileHandle == NULL)
    {
        /* Error: Unable to add the profile. Return the error code back */
        return errCode;
    }

    /* Record the profile: */
    ptrBaseCfgProfileHandle[RECEIVE_PROFILE_NUMBER] = profileHandle;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // chirpCfg           chirpCfg 0 0 0 0 0 0 0 0
    //<startIdx>
    //<endIdx>
    //<profileId>
    //<startFreqVar>         0       % Hz
    //<freqSlopeVar>         0       % KHz/us
    //<idleTimeVar>          0       % microseconds
    //<adcStartTimeVar>      0       % microseconds
    //<txEnable>             1       % b0 Enable TX0, b1 Enable TX1, b2 Enable TX2

    /* Initialize the chirp configuration: */
    memset ((void *)&chirpCfg, 0, sizeof(rlChirpCfg_t));

    /* Populate the chirp configuration: */
    chirpCfg.chirpStartIdx   = 0;   // The receive chirp is chirp zero
    chirpCfg.chirpEndIdx     = 0;
    chirpCfg.profileId       = RECEIVE_PROFILE_NUMBER;

    /* Translate from Hz to number of [1 LSB = (gCLI_mmwave_freq_scale_factor * 1e9) / 2^26 Hz]
     * units of mmwavelink format */
    chirpCfg.startFreqVar    = (uint32_t) ((float)0 * (1U << 26) /
                                            (gMmwMssMCB.rfFreqScaleFactor * 1e9));

    /* Translate from KHz/us to number of [1 LSB = (gCLI_mmwave_freq_scale_factor * 1e6) * 900 /2^26 KHz/us]
     * units of mmwavelink format */
    chirpCfg.freqSlopeVar    = (uint16_t) ((float)0 * (1U << 26) /
                                           ((gMmwMssMCB.rfFreqScaleFactor * 1e6) * 900.0));

    /* Translate from us to [1 LSB = 10ns] units of mmwavelink format */
    chirpCfg.idleTimeVar     = (uint32_t)((float)0 * 1000.0 / 10.0);

    /* Translate from us to [1 LSB = 10ns] units of mmwavelink format */
    chirpCfg.adcStartTimeVar = (uint32_t)((float)0 * 1000.0 / 10.0);

    chirpCfg.txEnable        = 0;

    /* Add the chirp to the profile */
    if (MMWave_addChirp (profileHandle, &chirpCfg, &errCode) == NULL)
    {
        /* Error: Unable to add the chirp. Return the error code. */
        return errCode;
    }

    // Ready for sensorStart
    return 0;
}

int32_t Ranging_ActivateReceiveConfiguration(float frequencyInGhz)
{

    /////////////////////////////////////////////////////////////////////////////////////////
    // Frame configuration      frameCfg 0 0 1 0 10 1 0
    //<chirpStartIdx>        0       % Start Index of Chirp Valid range = 0-511
    //<chirpEndIdx>          0       % End Index of Chirp Valid range = chirpStartIdx-511
    //<numLoops>             0       % Number of times to repeat from chirpStartIdx to chirpStartIdx in each frame, valid range = 1 to 255 \n
    //<numFrames>            0       % Number of frame to transmit Valid Range 0 to 65535 (0 for infinite frames) \n
    //<framePeriodicity>     0       % Frame repetition period milliseconds
    //<triggerSelect>        0       % 0x0001 SWTRIGGER (Software API based triggering), 0x0002 HWTRIGGER (Hardware SYNC_IN based triggering)
    //<frameTriggerDelay>

    /* Initialize the frame configuration: */
    memset ((void *)&gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg, 0, sizeof(rlFrameCfg_t));

    /* Populate the frame configuration: */
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.chirpStartIdx      = 0;          // chirpStartIdx
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.chirpEndIdx        = 0;          // chirpEndIdx
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.numLoops           = 1;          // numLoops
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.numFrames          = 0;          // numFrames
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.framePeriodicity   = (uint32_t)((float)10 * 1000000 / 5);  // framePeriodicity
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.triggerSelect      = 1;                                    // triggerSelect
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.frameTriggerDelay  = (uint32_t)((float)0 * 1000000 / 5);   // frameTriggerDelay

    // Ready for sensorStart
    return 0;
}


int32_t Ranging_SetTransmitConfiguration(float frequencyInGhz, uint8_t numGoldCodeBits, uint16_t goldCodePrn)
{
    rlProfileCfg_t          profileCfg;
    int32_t                 errCode;
    MMWave_ProfileHandle    profileHandle;
    MMWave_ProfileHandle*   ptrBaseCfgProfileHandle;
    rlChirpCfg_t            chirpCfg;
    gold_code_struct_t      goldCode;
    uint16_t                index;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Profile Config Zero          profileCfg 0 63.9494 3 0 6 0 0 0 0 64 12500 0 0 158

    if (gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode == MMWave_DFEDataOutputMode_FRAME)
    {
        ptrBaseCfgProfileHandle = &gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.profileHandle[0U];
    }
    else
    {
        ptrBaseCfgProfileHandle = &gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.profileHandle[0U];
    }

    // Check to see if a transmit profile is already there
    if(ptrBaseCfgProfileHandle[TRANSMIT_PROFILE_NUMBER] != NULL)
    {
        // Delete the profile
        // This also deletes all chirps associated with the profile
        if (MMWave_delProfile (gMmwMssMCB.ctrlHandle, ptrBaseCfgProfileHandle[TRANSMIT_PROFILE_NUMBER], &errCode))
        {
            /* Error: Unable to add the profile. Return the error code back */
            return errCode;
        }
    }

    /* Initialize the profile configuration: */
    memset ((void *)&profileCfg, 0, sizeof(rlProfileCfg_t));

    /* Populate the profile configuration: */
    profileCfg.profileId             = TRANSMIT_PROFILE_NUMBER;

    /* Translate from GHz to [1 LSB = gCLI_mmwave_freq_scale_factor * 1e9 / 2^26 Hz] units
     * of mmwavelink format */
    profileCfg.startFreqConst        = (uint32_t) (frequencyInGhz * (1U << 26) / gMmwMssMCB.rfFreqScaleFactor);

    /* Translate below times from us to [1 LSB = 10 ns] units of mmwavelink format */
    profileCfg.idleTimeConst         = (uint32_t)((float)3 * 1000 / 10);    // 3 us idle time
    profileCfg.adcStartTimeConst     = (uint32_t)((float)0 * 1000 / 10);    // 0 us ADC wait time
    profileCfg.rampEndTime           = (uint32_t)((float)6 * 1000 / 10);    // 6 us ramp time
    profileCfg.txOutPowerBackoffCode = 0;                                   // 0 dB Tx decrease
    profileCfg.txPhaseShifter        = 0;                                   // no phase shift
    profileCfg.freqSlopeConst        = 0;                                   // No slope
    profileCfg.txStartTime           = (int32_t)((float)0 * 1000 / 10);     // 10's of ns
    profileCfg.numAdcSamples         = 64;                                  // 5.12 us @ 12.5 MSPS
    profileCfg.digOutSampleRate      = 12500;                               // KSPS
    profileCfg.hpfCornerFreq1        = 0;                                   // 175 kHz - hpfCornerFreq1
    profileCfg.hpfCornerFreq2        = 0;                                   // 350 kHz - hpfCornerFreq2
    profileCfg.rxGain                = 158;                                 // 36 dB Rx Gain

    /* Add the profile to the mmWave module: */
    profileHandle = MMWave_addProfile (gMmwMssMCB.ctrlHandle, &profileCfg, &errCode);
    if (profileHandle == NULL)
    {
        /* Error: Unable to add the profile. Return the error code back */
        return errCode;
    }

    /* Record the profile: */
    ptrBaseCfgProfileHandle[TRANSMIT_PROFILE_NUMBER] = profileHandle;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // chirpCfg           chirpCfg 0 0 0 0 0 0 0 0
    //<startIdx>
    //<endIdx>
    //<profileId>
    //<startFreqVar>         0       % Hz
    //<freqSlopeVar>         0       % KHz/us
    //<idleTimeVar>          0       % microseconds
    //<adcStartTimeVar>      0       % microseconds
    //<txEnable>             1       % b0 Enable TX0, b1 Enable TX1, b2 Enable TX2


    if( generate_one_gold_sequence(
            numGoldCodeBits,
            &goldCode,
            goldCodePrn))
    {
        System_printf("Error generating transmit gold code.\n");
        Ranging_debugAssert (0);
    }

    /* Initialize the chirp configuration: */
    memset ((void *)&chirpCfg, 0, sizeof(rlChirpCfg_t));

    for(index = 0; index < goldCode.length; index++)
    {
        chirpCfg.chirpStartIdx   = index + 1;
        chirpCfg.chirpEndIdx     = index + 1;
        chirpCfg.profileId       = TRANSMIT_PROFILE_NUMBER;

        /* Translate from Hz to number of [1 LSB = (gCLI_mmwave_freq_scale_factor * 1e9) / 2^26 Hz]
         * units of mmwavelink format */
        chirpCfg.startFreqVar    = (uint32_t) ((float)0 * (1U << 26) /
                                                (gMmwMssMCB.rfFreqScaleFactor * 1e9));

        /* Translate from KHz/us to number of [1 LSB = (gCLI_mmwave_freq_scale_factor * 1e6) * 900 /2^26 KHz/us]
         * units of mmwavelink format */
        chirpCfg.freqSlopeVar    = (uint16_t) ((float)0 * (1U << 26) /
                                               ((gMmwMssMCB.rfFreqScaleFactor * 1e6) * 900.0));

        /* Translate from us to [1 LSB = 10ns] units of mmwavelink format */
        chirpCfg.idleTimeVar     = (uint32_t)((float)0 * 1000.0 / 10.0);

        /* Translate from us to [1 LSB = 10ns] units of mmwavelink format */
        chirpCfg.adcStartTimeVar = (uint32_t)((float)0 * 1000.0 / 10.0);

        if(goldCode.data[index] == 1)
        {
            chirpCfg.txEnable        = 1;
        }
        else
        {
            chirpCfg.txEnable        = 0;
        }

        /* Add the chirp to the profile */
        if (MMWave_addChirp (profileHandle, &chirpCfg, &errCode) == NULL)
        {
            /* Error: Unable to add the chirp. Return the error code. */
            return errCode;
        }
    }
    return 0;
}

int32_t Ranging_ActivateTransmitConfiguration(float frequencyInGhz)
{

    /////////////////////////////////////////////////////////////////////////////////////////
    // Frame configuration      frameCfg 0 0 1 0 10 1 0
    //<chirpStartIdx>        0       % Start Index of Chirp Valid range = 0-511
    //<chirpEndIdx>          0       % End Index of Chirp Valid range = chirpStartIdx-511
    //<numLoops>             0       % Number of times to repeat from chirpStartIdx to chirpStartIdx in each frame, valid range = 1 to 255 \n
    //<numFrames>            0       % Number of frame to transmit Valid Range 0 to 65535 (0 for infinite frames) \n
    //<framePeriodicity>     0       % Frame repetition period milliseconds
    //<triggerSelect>        0       % 0x0001 SWTRIGGER (Software API based triggering), 0x0002 HWTRIGGER (Hardware SYNC_IN based triggering)
    //<frameTriggerDelay>

    /* Initialize the frame configuration: */
    memset ((void *)&gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg, 0, sizeof(rlFrameCfg_t));

    /* Populate the frame configuration: */
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.chirpStartIdx      = 1;          // chirpStartIdx
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.chirpEndIdx        = 64;         // chirpEndIdx
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.numLoops           = 1;          // numLoops
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.numFrames          = 0;          // numFrames
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.framePeriodicity   = (uint32_t)((float)10 * 1000000 / 5);  // framePeriodicity
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.triggerSelect      = 1;                                    // triggerSelect
    gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.frameTriggerDelay  = (uint32_t)((float)0 * 1000000 / 5);   // frameTriggerDelay

    // Ready for sensorStart
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor stop command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLISensorStop (int32_t argc, char* argv[])
{
    if ((gMmwMssMCB.sensorState == Ranging_SensorState_STOPPED) ||
        (gMmwMssMCB.sensorState == Ranging_SensorState_INIT) ||
        (gMmwMssMCB.sensorState == Ranging_SensorState_OPENED))
    {
        CLI_write ("Ignored: Sensor is already stopped\n");
        return 0;
    }

    Ranging_stopSensor();

    Ranging_resetStaticCfgPendingState();

    gMmwMssMCB.sensorState = Ranging_SensorState_STOPPED;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function to get sub-frame number
 *
 *  @param[in] argc  Number of arguments
 *  @param[in] argv  Arguments
 *  @param[in] expectedArgc Expected number of arguments
 *  @param[out] subFrameNum Sub-frame Number (0 based)
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLIGetSubframe (int32_t argc, char* argv[], int32_t expectedArgc,
                                       int8_t* subFrameNum)
{
    int8_t subframe;
    
    /* Sanity Check: Minimum argument check */
    if (argc != expectedArgc)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /*Subframe info is always in position 1*/
    subframe = (int8_t) atoi(argv[1]);

    if(subframe >= (int8_t)RL_MAX_SUBFRAMES)
    {
        CLI_write ("Error: Subframe number is invalid\n");
        return -1;
    }

    *subFrameNum = (int8_t)subframe;

    return 0;
}



/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for gui monitoring configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLIGuiMonSel (int32_t argc, char* argv[])
{
    Ranging_GuiMonSel   guiMonSel;
    int8_t              subFrameNum;

    if(Ranging_CLIGetSubframe(argc, argv, 8, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize the guiMonSel configuration: */
    memset ((void *)&guiMonSel, 0, sizeof(Ranging_GuiMonSel));

    /* Populate configuration: */
    //guiMonSel.detectedObjects           = atoi (argv[2]);
    //guiMonSel.logMagRange               = atoi (argv[3]);
    guiMonSel.noiseProfile              = atoi (argv[4]);
    //guiMonSel.rangeAzimuthHeatMap       = atoi (argv[5]);
    //guiMonSel.rangeDopplerHeatMap       = atoi (argv[6]);
    guiMonSel.statsInfo                 = atoi (argv[7]);

    Ranging_CfgUpdate((void *)&guiMonSel, MMWDEMO_GUIMONSEL_OFFSET,
        sizeof(Ranging_GuiMonSel), subFrameNum);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for data logger set command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLIADCBufCfg (int32_t argc, char* argv[])
{
    Ranging_ADCBufCfg   adcBufCfg;
    int8_t              subFrameNum;

    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    if(Ranging_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&adcBufCfg, 0, sizeof(adcBufCfg));

    /* Populate configuration: */
    adcBufCfg.adcFmt          = (uint8_t) atoi (argv[2]);
    adcBufCfg.iqSwapSel       = (uint8_t) atoi (argv[3]);
    adcBufCfg.chInterleave    = (uint8_t) atoi (argv[4]);
    adcBufCfg.chirpThreshold  = (uint8_t) atoi (argv[5]);

    /* This demo is using HWA for 1D processing which does not allow multi-chirp
     * processing */
    if (adcBufCfg.chirpThreshold != 1)
    {
        CLI_write("Error: chirpThreshold must be 1, multi-chirp is not allowed\n");
        return -1;
    }
    /* Save Configuration to use later */
    Ranging_CfgUpdate((void *)&adcBufCfg,
                      MMWDEMO_ADCBUFCFG_OFFSET,
                      sizeof(Ranging_ADCBufCfg), subFrameNum);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for BPM configuration supported by the mmw Demo
 *      Note that there is a generic BPM configuration command supported by
 *      utils/cli and mmwave. The generic BPM command is not supported by the
 *      demo as the mmw demo assumes a specific BPM pattern for the TX antennas.
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLIBpmCfg (int32_t argc, char* argv[])
{
    int8_t              subFrameNum;
    Ranging_BpmCfg      bpmCfg;

    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    if(Ranging_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&bpmCfg, 0, sizeof(Ranging_BpmCfg));

    /* Populate configuration: */
    bpmCfg.isEnabled = (bool) atoi(argv[2]) ;
    bpmCfg.chirp0Idx = (uint16_t) atoi(argv[3]) ;
    bpmCfg.chirp1Idx = (uint16_t) atoi(argv[4]) ;

    /* Save Configuration to use later */
    Ranging_CfgUpdate((void *)&bpmCfg, MMWDEMO_BPMCFG_OFFSET,
                      sizeof(Ranging_BpmCfg), subFrameNum);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for configuring CQ RX Saturation monitor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLIChirpQualityRxSatMonCfg (int32_t argc, char* argv[])
{
    rlRxSatMonConf_t        cqSatMonCfg;

    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cqSatMonCfg, 0, sizeof(rlRxSatMonConf_t));

    /* Populate configuration: */
    cqSatMonCfg.profileIndx                 = (uint8_t) atoi (argv[1]);

    if(cqSatMonCfg.profileIndx < RL_MAX_PROFILES_CNT)
    {

        cqSatMonCfg.satMonSel                   = (uint8_t) atoi (argv[2]);
        cqSatMonCfg.primarySliceDuration        = (uint16_t) atoi (argv[3]);
        cqSatMonCfg.numSlices                   = (uint16_t) atoi (argv[4]);
        cqSatMonCfg.rxChannelMask               = (uint8_t) atoi (argv[5]);

        /* Save Configuration to use later */
        gMmwMssMCB.cqSatMonCfg[cqSatMonCfg.profileIndx] = cqSatMonCfg;

        return 0;
    }
    else
    {
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for configuring CQ Signal & Image band monitor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLIChirpQualitySigImgMonCfg (int32_t argc, char* argv[])
{
    rlSigImgMonConf_t       cqSigImgMonCfg;

    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cqSigImgMonCfg, 0, sizeof(rlSigImgMonConf_t));

    /* Populate configuration: */
    cqSigImgMonCfg.profileIndx              = (uint8_t) atoi (argv[1]);

    if(cqSigImgMonCfg.profileIndx < RL_MAX_PROFILES_CNT)
    {
        cqSigImgMonCfg.numSlices            = (uint8_t) atoi (argv[2]);
        cqSigImgMonCfg.timeSliceNumSamples  = (uint16_t) atoi (argv[3]);

        /* Save Configuration to use later */
        gMmwMssMCB.cqSigImgMonCfg[cqSigImgMonCfg.profileIndx] = cqSigImgMonCfg;

        return 0;
    }
    else
    {
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for enabling analog monitors
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLIAnalogMonitorCfg (int32_t argc, char* argv[])
{
    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Save Configuration to use later */
    gMmwMssMCB.anaMonCfg.rxSatMonEn = atoi (argv[1]);
    gMmwMssMCB.anaMonCfg.sigImgMonEn = atoi (argv[2]);

    gMmwMssMCB.isAnaMonCfgPending = 1;

    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the High Speed Interface
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLILvdsStreamCfg (int32_t argc, char* argv[])
{
    Ranging_LvdsStreamCfg   cfg;
    int8_t                  subFrameNum;

    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    if(Ranging_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(Ranging_LvdsStreamCfg));

    /* Populate configuration: */
    cfg.isHeaderEnabled = (bool)    atoi(argv[2]);
    cfg.dataFmt         = (uint8_t) atoi(argv[3]);
    cfg.isSwEnabled     = (bool)    atoi(argv[4]);

    /* If both h/w and s/w are enabled, HSI header must be enabled, because
     * we don't allow mixed h/w session without HSI header
     * simultaneously with s/w session with HSI header (s/w session always
     * streams HSI header) */
    if ((cfg.isSwEnabled == true) && (cfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED))
    {
        if (cfg.isHeaderEnabled == false)
        {
            CLI_write("Error: header must be enabled when both h/w and s/w streaming are enabled\n");
            return -1;
        }
    }

    /* Save Configuration to use later */
    Ranging_CfgUpdate((void *)&cfg,
                      MMWDEMO_LVDSSTREAMCFG_OFFSET,
                      sizeof(Ranging_LvdsStreamCfg), subFrameNum);

    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for configuring the data port
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLIConfigDataPort (int32_t argc, char* argv[])
{
    uint32_t baudrate;
    bool  ackPing;
    UART_Params uartParams;
    uint8_t ackData[16];
    

    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Populate configuration: */
    baudrate = (uint32_t) atoi(argv[1]);
    ackPing = (bool) atoi(argv[2]);

    /* check if requested value is less than max supported value */
    if (baudrate > MMWDEMO_DATAUART_MAX_BAUDRATE_SUPPORTED)
    {
        CLI_write ("Ignored: Invalid baud rate (%d) specified\n",baudrate);
        return 0;
    }

    /* re-open UART port if requested baud rate is different than current */
    if (gMmwMssMCB.cfg.platformCfg.loggingBaudRate != baudrate)
    {
        /* close previous opened handle */
        /* since the sensor is not running at this time, it is safe to close
           the existing port */
        if (gMmwMssMCB.loggingUartHandle != NULL)
        {
            UART_close(gMmwMssMCB.loggingUartHandle);
            gMmwMssMCB.loggingUartHandle = NULL;
        }   
        
        /* Setup the default UART Parameters */
        UART_Params_init(&uartParams);
        uartParams.writeDataMode  = UART_DATA_BINARY;
        uartParams.readDataMode   = UART_DATA_BINARY;
        uartParams.clockFrequency = gMmwMssMCB.cfg.platformCfg.sysClockFrequency;
        uartParams.baudRate       = baudrate;
        uartParams.isPinMuxDone   = 1U;

        /* Open the Logging UART Instance: */
        gMmwMssMCB.loggingUartHandle = UART_open(1, &uartParams);
        if (gMmwMssMCB.loggingUartHandle == NULL)
        {
            CLI_write ("Error: Unable to open the Logging UART Instance\n");
            return -1;
        }
        gMmwMssMCB.cfg.platformCfg.loggingBaudRate = baudrate;
        CLI_write ("Data port baud rate changed to %d\n",baudrate);
    }

    /* regardless of baud rate update, ack back to the host over this UART 
       port if handle is valid and user has requested the ack back */
    if ((gMmwMssMCB.loggingUartHandle != NULL) && (ackPing == true))
    {
        memset(ackData,0xFF,sizeof(ackData));
        UART_writePolling (gMmwMssMCB.loggingUartHandle,
                           (uint8_t*)ackData,
                           sizeof(ackData));
    }

    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for querying Demo status
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLIQueryDemoStatus (int32_t argc, char* argv[])
{
    CLI_write ("Sensor State: %d\n",gMmwMssMCB.sensorState);
    CLI_write ("Data port baud rate: %d\n",gMmwMssMCB.cfg.platformCfg.loggingBaudRate);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for save/restore calibration data to/from flash
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLICalibDataSaveRestore(int32_t argc, char* argv[])
{
    if (gMmwMssMCB.sensorState == Ranging_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Validate inputs */
    if ( ((uint32_t) atoi(argv[1]) == 1) && ((uint32_t) atoi(argv[2] ) == 1))
    {
        CLI_write ("Error: Save and Restore can be enabled only one at a time\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.calibCfg.saveEnable = (uint32_t) atoi(argv[1]);
    gMmwMssMCB.calibCfg.restoreEnable = (uint32_t) atoi(argv[2]);
    sscanf(argv[3], "0x%x", &gMmwMssMCB.calibCfg.flashOffset);

    gMmwMssMCB.isCalibCfgPending = 1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
void Ranging_CLIInit (uint8_t taskPriority)
{
    CLI_Cfg     cliCfg;
    char        demoBanner[256];
    uint32_t    cnt;

    /* Create Demo Banner to be printed out by CLI */
    sprintf(&demoBanner[0], 
                       "******************************************\n" \
                       "xWR68xx MMW Demo %02d.%02d.%02d.%02d\n"  \
                       "******************************************\n", 
                        MMWAVE_SDK_VERSION_MAJOR,
                        MMWAVE_SDK_VERSION_MINOR,
                        MMWAVE_SDK_VERSION_BUGFIX,
                        MMWAVE_SDK_VERSION_BUILD
            );

    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /* Populate the CLI configuration: */
    cliCfg.cliPrompt                    = "mmwDemo:/>";
    cliCfg.cliBanner                    = demoBanner;
    cliCfg.cliUartHandle                = gMmwMssMCB.commandUartHandle;
    cliCfg.taskPriority                 = taskPriority;
    cliCfg.socHandle                    = gMmwMssMCB.socHandle;
    cliCfg.mmWaveHandle                 = gMmwMssMCB.ctrlHandle;
    cliCfg.enableMMWaveExtension        = 1U;
    cliCfg.usePolledMode                = true;
    cliCfg.overridePlatform             = false;
    cliCfg.overridePlatformString       = NULL;    
    
    cnt=0;
    cliCfg.tableEntry[cnt].cmd            = "sensorStart";
    cliCfg.tableEntry[cnt].helpString     = "[doReconfig(optional, default:enabled)]";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLISensorStart;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "sensorStop";
    cliCfg.tableEntry[cnt].helpString     = "No arguments";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLISensorStop;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "guiMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <detectedObjects> <logMagRange> <noiseProfile> <rangeAzimuthHeatMap> <rangeDopplerHeatMap> <statsInfo>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLIGuiMonSel;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "adcbufCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <adcOutputFmt> <SampleSwap> <ChanInterleave> <ChirpThreshold>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLIADCBufCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "bpmCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enabled> <chirp0Idx> <chirp1Idx>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLIBpmCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQRxSatMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <satMonSel> <priSliceDuration> <numSlices> <rxChanMask>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLIChirpQualityRxSatMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQSigImgMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <numSlices> <numSamplePerSlice>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLIChirpQualitySigImgMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "analogMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<rxSaturation> <sigImgBand>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLIAnalogMonitorCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "lvdsStreamCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enableHeader> <dataFmt> <enableSW>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLILvdsStreamCfg;
    cnt++;
 
    cliCfg.tableEntry[cnt].cmd            = "configDataPort";
    cliCfg.tableEntry[cnt].helpString     = "<baudrate> <ackPing>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLIConfigDataPort;
    cnt++;
    
    cliCfg.tableEntry[cnt].cmd            = "queryDemoStatus";
    cliCfg.tableEntry[cnt].helpString     = "";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLIQueryDemoStatus;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "calibData";
    cliCfg.tableEntry[cnt].helpString    = "<save enable> <restore enable> <Flash offset>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLICalibDataSaveRestore;
    cnt++;

#ifdef SYS_COMMON_XWR68XX_LOW_POWER_MODE_EN
    cliCfg.tableEntry[cnt].cmd            = "idlePowerCycle";
    cliCfg.tableEntry[cnt].helpString     = "<enDSPpowerdown> <enDSSclkgate> <enMSSvclkgate> <enBSSclkgate> <enRFpowerdown> <enAPLLpowerdown> <enAPLLGPADCpowerdown> <componentMicroDelay> <idleModeMicroDelay> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLIIdleModeCycle;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "idlePowerDown";
    cliCfg.tableEntry[cnt].helpString     = "<enDSPpowerdown> <enDSSclkgate> <enMSSvclkgate> <enBSSclkgate> <enRFpowerdown> <enAPLLpowerdown> <enAPLLGPADCpowerdown> <componentMicroDelay> <idleModeMicroDelay> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = Ranging_CLIIdleModeDown;
    cnt++;
#endif /* SYS_COMMON_XWR68XX_LOW_POWER_MODE_EN */

    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        System_printf ("Error: Unable to open the CLI\n");
        return;
    }
    System_printf ("Debug: CLI is operational\n");
    return;
}

#ifdef SYS_COMMON_XWR68XX_LOW_POWER_MODE_EN
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the Idle Mode Power Cyle Function.
 *
 *      Function first takes in parameters corresponding to idle mode components
 *      desired by user then performs the idle_power_cycle() function which
 *      brings the device into an Idle Power state for a user-defined amount time
 *      and then brings the device back up into a normal powered state with CLI
 *      functionality.
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLIIdleModeCycle (int32_t argc, char* argv[])
{

    //First take parameters set at CLI
    IdleModeCfg   idleModeCfg;
    int8_t              subFrameNum;

    if(Ranging_CLIGetSubframe(argc, argv, 11, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&idleModeCfg, 0, sizeof(idleModeCfg));

    /* Populate configuration: */
    idleModeCfg.enDSPpowerdown      = (int8_t) atoi (argv[2]);
    idleModeCfg.enDSSclkgate      = (int8_t) atoi (argv[3]);
    idleModeCfg.enMSSvclkgate    = (int8_t) atoi (argv[4]);
    idleModeCfg.enBSSclkgate    = (int8_t) atoi (argv[5]);
    idleModeCfg.enRFpowerdown    = (int8_t) atoi (argv[6]);
    idleModeCfg.enAPLLpowerdown    = (int8_t) atoi (argv[7]);
    idleModeCfg.enAPLLGPADCpowerdown    = (int8_t) atoi (argv[8]);
    idleModeCfg.componentMicroDelay    = (uint32_t) atoi (argv[9]);
    idleModeCfg.idleModeMicroDelay    = (uint32_t) atoi (argv[10]);



    //Then, stop sensor and go into idle mode
    if ((gMmwMssMCB.sensorState == Ranging_SensorState_STOPPED) ||
          (gMmwMssMCB.sensorState == Ranging_SensorState_INIT) ||
          (gMmwMssMCB.sensorState == Ranging_SensorState_OPENED))
      {
          CLI_write ("Ignored: Sensor is already stopped\n");
          return 0;
      }

      Ranging_stopSensor();

      Ranging_resetStaticCfgPendingState();

      gMmwMssMCB.sensorState = Ranging_SensorState_STOPPED;


      idle_power_cycle(idleModeCfg);


    return 0;
}




/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the Idle Mode Power Down Function.
 *
 *      Function first takes in parameters corresponding to idle mode components
 *      desired by user then performs the idle_power_down() function which
 *      brings the device into an Idle Power state where it remains indefinitely.
 *      This function does NOT bring the device back into a normal power state and
 *      a hard reset of the device is required to do so.
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Ranging_CLIIdleModeDown (int32_t argc, char* argv[])
{

    //First take parameters set at CLI
    IdleModeCfg   idleModeCfg;
    int8_t              subFrameNum;

    if(Ranging_CLIGetSubframe(argc, argv, 11, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&idleModeCfg, 0, sizeof(idleModeCfg));

    /* Populate configuration: */
    idleModeCfg.enDSPpowerdown      = (int8_t) atoi (argv[2]);
    idleModeCfg.enDSSclkgate      = (int8_t) atoi (argv[3]);
    idleModeCfg.enMSSvclkgate    = (int8_t) atoi (argv[4]);
    idleModeCfg.enBSSclkgate    = (int8_t) atoi (argv[5]);
    idleModeCfg.enRFpowerdown    = (int8_t) atoi (argv[6]);
    idleModeCfg.enAPLLpowerdown    = (int8_t) atoi (argv[7]);
    idleModeCfg.enAPLLGPADCpowerdown    = (int8_t) atoi (argv[8]);
    idleModeCfg.componentMicroDelay    = (uint32_t) atoi (argv[9]);
    idleModeCfg.idleModeMicroDelay    = (uint32_t) atoi (argv[10]);


    //Then, stop sensor and go into idle mode
    if ((gMmwMssMCB.sensorState == Ranging_SensorState_STOPPED) ||
          (gMmwMssMCB.sensorState == Ranging_SensorState_INIT) ||
          (gMmwMssMCB.sensorState == Ranging_SensorState_OPENED))
      {
          CLI_write ("Ignored: Sensor is already stopped\n");
          return 0;
      }

      Ranging_stopSensor();

      Ranging_resetStaticCfgPendingState();

      gMmwMssMCB.sensorState = Ranging_SensorState_STOPPED;


      idle_power_down(idleModeCfg);


    return 0;
}
#endif /* SYS_COMMON_XWR68XX_LOW_POWER_MODE_EN */
