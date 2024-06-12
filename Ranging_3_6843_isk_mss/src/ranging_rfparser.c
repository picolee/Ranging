/**
 *   @file  mmwdemo_rfparser.c
 *
 *   @brief
 *      Implements rf parser.
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
#include <math.h>

#ifdef RANGING_RFPARSER_DBG
/* enable float extended format in BIOS cfg file using System.extendedFormats
   to able to print %f values */
#include <xdc/runtime/System.h>
#endif

/* mmWave SDK Include files */
#include <ti/common/sys_common.h>
#include <inc/ranging_rfparser.h>

/* MATH utils library Include files */
#include <ti/utils/mathutils/mathutils.h>

/** @defgroup RANGING_RFPARSER_INTERNAL       Mmwdemo RFparser Internal
 */

/**
@defgroup RANGING_RFPARSER_INTERNAL_FUNCTION            RF Parser Internal Functions
@ingroup RANGING_RFPARSER_INTERNAL
@brief
*   The section has a list of all internal API which are not exposed to the external
*   applications.
*/
/**
@defgroup RANGING_RFPARSER_INTERNAL_DATA_STRUCTURE      RF Parser Internal Data Structures
@ingroup RANGING_RFPARSER_INTERNAL
@brief
*   The section has a list of all internal data structures which are used internally
*   by the RF Parser module.
*/
/**
@defgroup RANGING_RFPARSER_INTERNAL_DEFINITION          RF Parser Internal Definitions
@ingroup RANGING_RFPARSER_INTERNAL
@brief
*   The section has a list of all internal definitions which are used internally
*   by the RF Parser.
*/


/** @addtogroup RANGING_RFPARSER_INTERNAL_DEFINITION
 *
 @{ */

/*! Speed of light in m/s expressed in float */
#define RANGING_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC (3.0e8)

/** @} */

/**
 * @brief
 *  Data Source Hardware configuration definition
 *
 * @details
 *  The structure describes the hardware related configuration for device and evm
 *
 *  \ingroup RANGING_RFPARSER_INTERNAL_DATA_STRUCTURE
 */
typedef struct Ranging_RFParserHwAttr_t
{
    /**
     * @brief   ADC buffer size
     */
    uint32_t      adcBufSize;

    /**
     * @brief   Tx Antenna mask for elevation, antenna pattern specific
     */
    uint8_t       elevTxAntMask;

    /**
     * @brief   Tx Antenna mask for azimuth, antenna pattern specific
     */
    uint8_t       azimTxAntMask;
} Ranging_RFParserHwAttr;

/*================================================================
               RF Parser platform dependent params
 ================================================================*/
#ifdef SOC_XWR14XX
Ranging_RFParserHwAttr Ranging_RFParserHwCfg =
{
    SOC_XWR14XX_MSS_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#ifdef SOC_XWR16XX
#ifdef SUBSYS_DSS
Ranging_RFParserHwAttr Ranging_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0,
    0x3
};
#endif

#ifdef SUBSYS_MSS
Ranging_RFParserHwAttr Ranging_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0,
    0x3
};
#endif

#endif

#ifdef SOC_XWR18XX

#ifdef SUBSYS_MSS
Ranging_RFParserHwAttr Ranging_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#ifdef SUBSYS_DSS
Ranging_RFParserHwAttr Ranging_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#endif

#ifdef SOC_XWR68XX

#ifdef SUBSYS_MSS
Ranging_RFParserHwAttr Ranging_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#ifdef SUBSYS_DSS
Ranging_RFParserHwAttr Ranging_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#endif
/*================================================================
               RF Parser internal APIs
 ================================================================*/

/**
 *  @b Description
 *  @n
 *      Help Function to get frame period
 *
 *  @param[in] ctrlCfg       Handle to MMWave control configuration
 *  @param[in] subFrameIndx  Sub frame index
 *
 *  \ingroup RANGING_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Frame period
 */
static float Ranging_RFParser_getFramePeriod_ms(MMWave_CtrlCfg *ctrlCfg, uint8_t subFrameIndx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].subFramePeriodicity * 0.000005f);
    }
    else
    {
        return((float)ctrlCfg->u.frameCfg.frameCfg.framePeriodicity * 0.000005f);
    }
}


/**
 *  @b Description
 *  @n
 *      Help Function to get chirp start Index
 *
 *  @param[in] ctrlCfg       Handle to MMWave control configuration
 *  @param[in] subFrameIndx  Sub frame index
 *
 *  \ingroup RANGING_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Chirp start index
 */
static uint16_t Ranging_RFParser_getChirpStartIdx(MMWave_CtrlCfg *ctrlCfg, uint8_t subFrameIndx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].chirpStartIdx);
    }
    else
    {
        return(ctrlCfg->u.frameCfg.frameCfg.chirpStartIdx);
    }
}

/**
 *  @b Description
 *  @n
 *      Help Function to get chirp stop Index
 *
 *  @param[in] ctrlCfg       Handle to MMWave control configuration
 *  @param[in] subFrameIndx  Sub frame index
 *
 *  \ingroup RANGING_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Chirp stop index
 */
static uint16_t Ranging_RFParser_getChirpEndIdx(MMWave_CtrlCfg *ctrlCfg, uint8_t subFrameIndx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].chirpStartIdx +
              (ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].numOfChirps - 1));
    }
    else
    {
        return(ctrlCfg->u.frameCfg.frameCfg.chirpEndIdx);
    }
}

/**
 *  @b Description
 *  @n
 *      Help Function to get number of loops
 *
 *  @param[in] ctrlCfg       Handle to MMWave control configuration
 *  @param[in] subFrameIndx  Sub frame index
 *
 *  \ingroup RANGING_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Number of loops
 */
static uint16_t Ranging_RFParser_getNumLoops(MMWave_CtrlCfg *ctrlCfg, uint8_t subFrameIndx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].numLoops);
    }
    else
    {
        return(ctrlCfg->u.frameCfg.frameCfg.numLoops);
    }
}

/**
 *  @b Description
 *  @n
 *      Help Function to get profile handle
 *
 *  @param[in] ctrlCfg          Handle to MMWave control configuration
 *  @param[in] profileLoopIdx   Profile index
 *
 *  \ingroup RANGING_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Profile handle
 */
static MMWave_ProfileHandle Ranging_RFParser_getProfileHandle(MMWave_CtrlCfg *ctrlCfg, uint32_t profileLoopIdx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.profileHandle[profileLoopIdx]);
    }
    else
    {
        return(ctrlCfg->u.frameCfg.profileHandle[profileLoopIdx]);
    }
}

/**
 *  @b Description
 *  @n
 *      Helper function that parses Profile, Chirp and Frame config and extracts parameters
 *      needed for processing chain configuration
 *
 *  @param[out] outParams      Pointer to parameters generated after parsing configuration
 *  @param[in]  subFrameIdx    Sub-frame index
 *  @param[in]  openCfg              Pointer to the MMWave Open configuration
 *  @param[in]  ctrlCfg              Pointer to the MMWave Control configuration
 *  @param[in]  rfFreqScaleFactor RF frequency scale factor, see SOC_getDeviceRFFreqScaleFactor API
 *                                in SOC driver
 *  @param[in]  bpmEnabled     BPM flag, 0 -disabled, 1-enabled
 *
 *  \ingroup RANGING_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *      Fail        <0, one of error codes @ref RANGING_RFPARSER_ERROR_CODE
 */
static int32_t Ranging_RFParser_parseCtrlConfig
(
    Ranging_RFParserOutParams  *outParams,
    uint8_t              subFrameIdx,
    MMWave_OpenCfg      *openCfg,
    MMWave_CtrlCfg      *ctrlCfg,
    float               rfFreqScaleFactor,
    bool                bpmEnabled
)
{
    uint16_t    frameChirpStartIdx;
    uint16_t    frameChirpEndIdx;
    int16_t     frameTotalChirps;
    uint32_t    profileLoopIdx;
    bool        foundValidProfile = false;
    uint8_t     channel;
    uint8_t     numRxAntennas;
    uint16_t    numLoops;
    int32_t     retVal = 0;
    int32_t     errCode;
    float       bandwidth, centerFreq, adcStart, slope, startFreq;

    /***********************************************
     * Sanity Check on ADC configuration
     ***********************************************/
    /* Only support 16 Bits ADC out bits */
    if(openCfg->adcOutCfg.fmt.b2AdcBits != 2U)
    {
        retVal = RANGING_RFPARSER_ENOTSUPPORT__NON_16BITS_ADC;
        goto exit;
    }

    /* Only support complex ADC data format */
    if((openCfg->adcOutCfg.fmt.b2AdcOutFmt != 1U) &&
       (openCfg->adcOutCfg.fmt.b2AdcOutFmt != 2U))
    {
        retVal = RANGING_RFPARSER_ENOTSUPPORT__NON_COMPLEX_ADC_FORMAT;
        goto exit;
    }

    /***********************************************
     * Parse openCfg
     ***********************************************/
    /* Find number of enabled channels */
    numRxAntennas = 0U;
    for (channel = 0U; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        if(openCfg->chCfg.rxChannelEn & (0x1U << channel))
        {
            outParams->rxAntOrder[numRxAntennas] = channel;
            /* Track the number of receive channels: */
            numRxAntennas++;
        }
        else
        {
            outParams->rxAntOrder[channel] = 0U;
        }

        //add check to make sure lambda/2
        //check for rxAnt -= 1, 2, 4 and their limitations
    }



    /***********************************************
     * Parse frameCfg
     ***********************************************/
    /* Read frameCfg chirp start/stop index */
    frameChirpStartIdx  = Ranging_RFParser_getChirpStartIdx(ctrlCfg, subFrameIdx);
    frameChirpEndIdx    = Ranging_RFParser_getChirpEndIdx(ctrlCfg, subFrameIdx);
    numLoops            = Ranging_RFParser_getNumLoops(ctrlCfg, subFrameIdx);
    outParams->framePeriod  =  Ranging_RFParser_getFramePeriod_ms(ctrlCfg, subFrameIdx);
    frameTotalChirps    = frameChirpEndIdx - frameChirpStartIdx + 1;

    /* TODO::Advance Frame only support one burst - chain limitation,  */
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        if(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIdx].numOfBurst != 1)
        {
            retVal = RANGING_RFPARSER_ENOTSUPPORT__NON_ONE_NUMOFBURST_FOR_ADVANCED_FRAME;
            goto exit;
        }
    }

    /* since validChirpTxEnBits is static array of 32 */
    if(frameTotalChirps > 32U)
    {
        retVal = RANGING_RFPARSER_ENOIMPL__NUM_UNIQUE_CHIRPS_MORE_THAN_32;
    }

    /* loop for profiles and find if it has valid chirps */
    /* we support only one profile in this processing chain */
    for (profileLoopIdx = 0;
        ((profileLoopIdx < MMWAVE_MAX_PROFILE) && (foundValidProfile == false));
        profileLoopIdx++)
    {
        uint32_t    mmWaveNumChirps = 0;
        MMWave_ProfileHandle profileHandle;

        profileHandle = Ranging_RFParser_getProfileHandle(ctrlCfg, profileLoopIdx);
        if (profileHandle == NULL)
            continue; /* skip this profile */

        /* get numChirps for this profile; skip error checking */
        retVal = MMWave_getNumChirps(profileHandle, &mmWaveNumChirps, &errCode);
        if (retVal != 0)
        {
            retVal = errCode;
            goto exit;
        }

        if(mmWaveNumChirps > 0)
        {
            foundValidProfile = true;
        }

        /* found valid chirps for the frame; set remaining parameters */
        if (foundValidProfile == true) {
            rlProfileCfg_t  profileCfg;
            
            outParams->validProfileIdx = profileLoopIdx;
            /* Get the profile configuration: */
            retVal = MMWave_getProfileCfg(profileHandle,&profileCfg, &errCode);
            if (retVal != 0)
            {
                retVal = errCode;
                goto exit;
            }
            
#ifndef MMW_ENABLE_NEGATIVE_FREQ_SLOPE
            /* Check frequency slope */
            if (profileCfg.freqSlopeConst < 0)
            {
                retVal = RANGING_RFPARSER_ENOTSUPPORT__NEGATIVE_FREQ_SLOPE;
                goto exit;
            }
#endif
            /* set other parameters */
            outParams->numAdcSamples = profileCfg.numAdcSamples;
            outParams->numChirpsPerFrame = frameTotalChirps * numLoops;
            outParams->adcSampleRate = profileCfg.digOutSampleRate * 1e3;
            
            adcStart                        =   ((float)profileCfg.adcStartTimeConst * 10.f * 1.e-9);
            startFreq                       =   (float)profileCfg.startFreqConst/(float)(1U << 26)*rfFreqScaleFactor*(float)1e9;
            slope                           =   (float)profileCfg.freqSlopeConst * ((rfFreqScaleFactor*1e3*900.f)/(float)(1U << 26)) * 1e12;
            bandwidth                       =   (slope * outParams->numAdcSamples)/(profileCfg.digOutSampleRate * 1e3);            
            centerFreq                      =   startFreq + bandwidth * 0.5f + adcStart * slope;            
            outParams->chirpInterval        =   (float)(profileCfg.idleTimeConst + profileCfg.rampEndTime)/1000.*10*1.e-6;
            
            outParams->centerFreq           =   centerFreq;
            
#ifdef RANGING_RFPARSER_DBG
            System_printf("startFreqConst: %d\n", profileCfg.startFreqConst);
            System_printf("rfFreqScaleFactor: %f\n", rfFreqScaleFactor);
            System_printf("bandwidth : %f GHz\n", bandwidth*1.e-9);
            System_printf("adcStartTimeConst: %d\n", profileCfg.adcStartTimeConst);
            System_printf("freqSlopeConst: %d\n", profileCfg.freqSlopeConst);
            System_printf("centerFreq: %f GHz\n", centerFreq*1.e-9);
            System_printf("dopplerStep: %f\n", outParams->dopplerStep);
            System_printf("adcStart: %f us\n", adcStart * 1e6);
            System_printf("slope: %f GHz\n", slope*1.e-9);
            System_printf("startFreq: %f GHz\n", startFreq*1.e-9);
#endif
            
        }
    }

    if (foundValidProfile == false)
    {
        retVal = RANGING_RFPARSER_EINVAL__VALID_PROFILECFG_NOT_FOUND;
        goto exit;
    }

exit:
    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      Helper function that parses ADCBuf configuration to be used to configure ADCBuf driver
 *
 *  @param[out] outParams      Pointer to parameters generated after parsing configuration
 *  @param[in]  adcBufCfg            Pointer to ADCBuf configuration
 *
 *  \ingroup RANGING_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *      Fail        < 0, one of @ref RANGING_RFPARSER_ERROR_CODE
 */
static int32_t Ranging_RFParser_parseADCBufCfg
(
    Ranging_RFParserOutParams   *outParams,
    Ranging_ADCBufCfg     *adcBufCfg
)
{
    uint16_t            numBytePerSample = 0;
    uint32_t            chirpThreshold;
    uint32_t            maxChirpThreshold;
    uint32_t            bytesPerChirp;
    int32_t             retVal = 0;

    /* Only support Complex */
    if (adcBufCfg->adcFmt != 0)
    {
        retVal = RANGING_RFPARSER_ENOTSUPPORT__NONCOMPLEX_ADC_FORMAT;
        goto exit;
    }

    /* Complex dataFormat has 4 bytes */
    numBytePerSample = 4U;

    /* Calculate RX channel data size and make it align to 16 bytes */
    outParams->adcBufChanDataSize =  outParams->numAdcSamples * numBytePerSample;
    outParams->adcBufChanDataSize = (outParams->adcBufChanDataSize + 15U) / 16U * 16U;

    /* Calculate max possible chirp threshold */
    bytesPerChirp = outParams->adcBufChanDataSize;

    /* find maximum number of full chirps that can fit in the ADCBUF memory, while
       also being able to divide numChirpsPerFrame, we do not want remainder processing */
    maxChirpThreshold = Ranging_RFParserHwCfg.adcBufSize/ bytesPerChirp;
    if (maxChirpThreshold >= outParams->numChirpsPerFrame)
    {
        maxChirpThreshold = outParams->numChirpsPerFrame;
        if (maxChirpThreshold > SYS_COMMON_CQ_MAX_CHIRP_THRESHOLD)
        {
            /* If CQ monitor is enabled, then check max chirpthreshold */
            maxChirpThreshold = SYS_COMMON_CQ_MAX_CHIRP_THRESHOLD;
        }
    }
    else
    {
        /* Find largest divisor of numChirpsPerFrame no bigger than maxChirpThreshold */
        while (outParams->numChirpsPerFrame % maxChirpThreshold)
        {
            maxChirpThreshold--;
        }
    }

    /* ADCBuf control function requires argument alignment at 4 bytes boundary */
    chirpThreshold = adcBufCfg->chirpThreshold;

    /* if automatic, set to the calculated max */
    if (chirpThreshold == 0)
    {
        chirpThreshold = maxChirpThreshold;
    }
    else
    {
        if (chirpThreshold > maxChirpThreshold)
        {
#ifdef RANGING_RFPARSER_DBG
            System_printf("Desired chirpThreshold %d higher than max possible of %d, setting to max\n",
                chirpThreshold, maxChirpThreshold);
#endif
            retVal = RANGING_RFPARSER_EINVAL__CHIRP_THRESH_GREATER_THAN_MAX_ALLOWED;
            goto exit;
        }
        else
        {
            /* check for divisibility of the user provided threshold */
            if((outParams->numChirpsPerFrame % chirpThreshold) != 0)
            {
                retVal = RANGING_RFPARSER_ENOTSUPPORT__NON_DIVISIBILITY_OF_CHIRP_THRESH;
                goto exit;
            }
            else
            {
                /* Chirp threshold has a valid configuration */
            }
        }
    }

#ifdef RANGING_RFPARSER_DBG
    System_printf(" chirpThreshold %d max = %d, \n",
                  chirpThreshold, maxChirpThreshold);
#endif

    /* Save chirpThreshold */
    outParams->numChirpsPerChirpEvent = chirpThreshold;

exit:
    return (retVal);
}

/**************************************************************
 * Exposed APIs
 **************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to parse chirp/profile configurations and 
 *  save configuration and derived parameter in datapath parameter structure.
 *
 *  @param[out] outParams      Pointer to parameters generated after parsing configuration
 *  @param[in]  subFrameIdx    Sub-frame index
 *  @param[in]  openCfg        Pointer to the MMWave Open configuration
 *  @param[in]  ctrlCfg        Pointer to the MMWave Control configuration
 *  @param[in]  adcBufCfg      Pointer to ADCBuf configuration
 *  @param[in]  rfFreqScaleFactor RF frequency scale factor, see SOC_getDeviceRFFreqScaleFactor API
 *  @param[in]  bpmEnable      BPM flag, 0 -disabled, 1-enabled
 *
 *  \ingroup RANGING_RFPARSER_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - = 0
 *  @retval
 *      Error       - < 0, one of @ref RANGING_RFPARSER_ERROR_CODE
 */
int32_t Ranging_RFParser_parseConfig
(
    Ranging_RFParserOutParams    *outParams,
    uint8_t              subFrameIdx,
    MMWave_OpenCfg       *openCfg,
    MMWave_CtrlCfg       *ctrlCfg,
    Ranging_ADCBufCfg    *adcBufCfg,
    float                rfFreqScaleFactor,
    bool                 bpmEnable
)
{
    int32_t retVal;

    if(subFrameIdx < RL_MAX_SUBFRAMES)
    {
        /* Parse the profile and chirp configs and get the valid number of TX Antennas */
        retVal = Ranging_RFParser_parseCtrlConfig(outParams, subFrameIdx,
                                                  openCfg, ctrlCfg, rfFreqScaleFactor, bpmEnable);

        if (retVal != 0)
        {
            goto exit;
        }
        retVal = Ranging_RFParser_parseADCBufCfg(outParams, adcBufCfg);
        if (retVal != 0)
        {
            goto exit;
        }
    }
    else
    {
        retVal = RANGING_RFPARSER_EINVAL__NUM_SUBFRAMES;
    }

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get number of sub-frames from the input
 *      chirp/profile configuration.
 *
 *  @param[in]  ctrlCfg        Pointer to the MMWave Control configuration
 *
 *  \ingroup RANGING_RFPARSER_EXTERNAL_FUNCTION
 *
 *  @retval  Number of Sub-frames
 */
uint8_t Ranging_RFParser_getNumSubFrames(MMWave_CtrlCfg *ctrlCfg)
{
    if(ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return (ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.numOfSubFrames);
    }
    else
    {
        return (1);
    }
}
