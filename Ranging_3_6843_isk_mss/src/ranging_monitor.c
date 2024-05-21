/**
 *   @file  mmwdemo_monitor.c
 *
 *   @brief
 *      The file implements the functions which are required to support
 *      monitoring functions.
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
#include <xdc/runtime/System.h>
#include <ti/common/sys_common.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/mathutils/mathutils.h>

#include <inc/ranging_monitor.h>
#include <inc/ranging_mss.h>

/**************************************************************************
 **************************** Globals  ************************************
 **************************************************************************/

extern Ranging_MSS_MCB    gMmwMssMCB;

/**************************************************************************
 **************************** Defies  ************************************
 **************************************************************************/

/* These address offsets are in bytes, when configure address offset in hardware,
   these values will be converted to number of 128bits
   Buffer at offset 0x0U is reserved by BSS, hence offset starts from 0x800
 */
#define MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET          0x800U
#define MMW_DEMO_CQ_RXSAT_ADDR_OFFSET           0x1000U

/* CQ data is at 16 bytes alignment for mulitple chirps */
#define MMW_DEMO_CQ_DATA_ALIGNMENT            16U

/**************************************************************************
 **************************** Local Functions *****************************
 **************************************************************************/

/**************************************************************************
 **************************** Monitor Functions *****************************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      The function is the exported API which is used to configure the mmWave analog monitor.
 *
 *  @param[in]  ptrAnaMonCfg
 *      Pointer to Analog Monitor configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t ranging_cfgAnalogMonitor(Ranging_AnaMonitorCfg         *ptrAnaMonCfg)
{
    rlMonAnaEnables_t   analogMonCfg;
    int32_t             retVal = 0;

    if (ptrAnaMonCfg == NULL) 
    {
        /* Error: Invalid argument. */
        retVal = MINUS_ONE;
    }
    else
    {
        /* Set config structure to all zeros */
        memset((void *)&analogMonCfg, 0, sizeof(rlMonAnaEnables_t));

        /* Set/Reset Rx saturation monitor bit */
        if(ptrAnaMonCfg->rxSatMonEn)
        {
            analogMonCfg.enMask  |= 0x1 << MMWDEMO_ANALOG_MONITOR_RX_SATURATION_DETECTOR;
        }

        /* Set/Reset Rx signal image band monitor bit */
        if(ptrAnaMonCfg->sigImgMonEn)
        {
            analogMonCfg.enMask  |= 0x1 << MMWDEMO_ANALOG_MONITOR_RX_SIG_IMG_BAND;
        }
        
        /* Send analog monitor config to sensor */
        retVal = rlRfAnaMonConfig(RL_DEVICE_MAP_INTERNAL_BSS, &analogMonCfg);
    }
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is the exported API which is used to configure the mmWave Rx Signal and
 *      image band energy monitor.
 *
 *  @param[in]  ptrSigImgMonCfg
 *      Pointer to the Signal & image band monitor configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t ranging_cfgRxSigImgMonitor
(
    rlSigImgMonConf_t*  ptrSigImgMonCfg
)
{
    int32_t             retVal = 0;

    /* Get the pointer to the control module */
    if ( (ptrSigImgMonCfg == NULL) || 
       (ptrSigImgMonCfg->numSlices < 1U) ||
       (ptrSigImgMonCfg->numSlices > RL_NUM_MON_SLICES_MAX) ||
       (ptrSigImgMonCfg->timeSliceNumSamples < 4U))
    {
        /* Error: Invalid argument. */
        retVal = MINUS_ONE;
        goto exit;
    }

    retVal = rlRfRxSigImgMonConfig(RL_DEVICE_MAP_INTERNAL_BSS, ptrSigImgMonCfg);
    if(retVal != 0)
    {
        /* Error: . */
        //System_printf ("Error: rlRfRxSigImgMonConfig returns error = %d\n", retVal);
    }

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is the exported API which is used to configure the mmWave Rx
 *      Saturation Monitor.
 *
 *  @param[in]  ptrRxSatMonCfg
 *      Pointer to the Rx Saturation monitor configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t ranging_cfgRxSaturationMonitor
(
    rlRxSatMonConf_t*   ptrRxSatMonCfg
)
{
    int32_t             retVal;

    /* Get the pointer to the control module */
    if ( ptrRxSatMonCfg == NULL )
    {
        /* Error: Invalid argument. */
        retVal = MINUS_ONE;
        goto exit;
    }

    /* Validate parameters */
    if( (ptrRxSatMonCfg->numSlices < 1U) ||
       (ptrRxSatMonCfg->numSlices > RL_NUM_MON_SLICES_MAX) ||
       (ptrRxSatMonCfg->satMonSel < 1U) ||
       (ptrRxSatMonCfg->satMonSel > 3U) ||
       (ptrRxSatMonCfg->primarySliceDuration < 4U) )
    {
        /* Error: Invalid argument. */
        retVal = MINUS_ONE;
        goto exit;
    }

    retVal = rlRfRxIfSatMonConfig(RL_DEVICE_MAP_INTERNAL_BSS, ptrRxSatMonCfg);
    if(retVal != 0)
    {
        //System_printf ("Error: rlRfRxIfSatMonConfig returns error = %d\n", retVal);
    }

exit:
    return retVal;
}



/**
 *  @b Description
 *  @n
 *      Function to configure CQ.
 *
 *  @param[in] subFrameCfg Pointer to sub-frame config
 *  @param[in] numChirpsPerChirpEvent number of chirps per chirp event
 *  @param[in] validProfileIdx valid profile index
 *
 *  @retval
 *      0 if no error, else error (there will be system prints for these).
 */
int32_t Ranging_configCQ(Ranging_SubFrameCfg *subFrameCfg,
                                uint8_t numChirpsPerChirpEvent,
                                uint8_t validProfileIdx)
{
    Ranging_AnaMonitorCfg*      ptrAnaMonitorCfg;
    ADCBuf_CQConf               cqConfig;
    rlRxSatMonConf_t*           ptrSatMonCfg;
    rlSigImgMonConf_t*          ptrSigImgMonCfg;
    int32_t                     retVal;
    uint16_t                    cqChirpSize;

    /* Get analog monitor configuration */
    ptrAnaMonitorCfg = &gMmwMssMCB.anaMonCfg;

    /* Config mmwaveLink to enable Saturation monitor - CQ2 */
    ptrSatMonCfg = &gMmwMssMCB.cqSatMonCfg[validProfileIdx];

    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        if (ptrSatMonCfg->profileIndx != validProfileIdx)
        {
            System_printf ("Error: Saturation monitoring (globally) enabled but not configured for profile(%d)\n",
                           validProfileIdx);
            Ranging_debugAssert(0);
        }

        retVal = ranging_cfgRxSaturationMonitor(ptrSatMonCfg);
        if(retVal != 0)
        {
            System_printf ("Error: rlRfRxIfSatMonConfig returns error = %d for profile(%d)\n",
                           retVal, ptrSatMonCfg->profileIndx);
            goto exit;
        }
    }

    /* Config mmwaveLink to enable Saturation monitor - CQ1 */
    ptrSigImgMonCfg = &gMmwMssMCB.cqSigImgMonCfg[validProfileIdx];

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        if (ptrSigImgMonCfg->profileIndx != validProfileIdx)
        {
            System_printf ("Error: Sig/Image monitoring (globally) enabled but not configured for profile(%d)\n",
                           validProfileIdx);
            Ranging_debugAssert(0);
        }

        retVal = ranging_cfgRxSigImgMonitor(ptrSigImgMonCfg);
        if(retVal != 0)
        {
            System_printf ("Error: rlRfRxSigImgMonConfig returns error = %d for profile(%d)\n",
                           retVal, ptrSigImgMonCfg->profileIndx);
            goto exit;
        }
    }

    retVal = ranging_cfgAnalogMonitor(ptrAnaMonitorCfg);
    if (retVal != 0)
    {
        System_printf ("Error: rlRfAnaMonConfig returns error = %d\n", retVal);
        goto exit;
    }

    if(ptrAnaMonitorCfg->rxSatMonEn || ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* CQ driver config */
        memset((void *)&cqConfig, 0, sizeof(ADCBuf_CQConf));
        cqConfig.cqDataWidth = 0; /* 16bit for mmw demo */
        cqConfig.cq1AddrOffset = MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET; /* CQ1 starts from the beginning of the buffer */
        cqConfig.cq2AddrOffset = MMW_DEMO_CQ_RXSAT_ADDR_OFFSET;  /* Address should be 16 bytes aligned */

        retVal = ADCBuf_control(gMmwMssMCB.adcBufHandle, ADCBufMMWave_CMD_CONF_CQ, (void *)&cqConfig);
        if (retVal < 0)
        {
            System_printf ("Error: MMWDemoDSS Unable to configure the CQ\n");
            Ranging_debugAssert(0);
        }
    }

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* This is for 16bit format in mmw demo, signal/image band data has 2 bytes/slice
           For other format, please check DFP interface document
         */
        cqChirpSize = (ptrSigImgMonCfg->numSlices + 1) * sizeof(uint16_t);
        cqChirpSize = MATHUTILS_ROUND_UP_UNSIGNED(cqChirpSize, MMW_DEMO_CQ_DATA_ALIGNMENT);
        subFrameCfg->sigImgMonTotalSize = cqChirpSize * numChirpsPerChirpEvent;
    }

    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        /* This is for 16bit format in mmw demo, saturation data has one byte/slice
           For other format, please check DFP interface document
         */
        cqChirpSize = (ptrSatMonCfg->numSlices + 1) * sizeof(uint8_t);
        cqChirpSize = MATHUTILS_ROUND_UP_UNSIGNED(cqChirpSize, MMW_DEMO_CQ_DATA_ALIGNMENT);
        subFrameCfg->satMonTotalSize = cqChirpSize * numChirpsPerChirpEvent;
    }

exit:
    return(retVal);
}
