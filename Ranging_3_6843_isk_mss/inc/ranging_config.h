/**
 *   @file  mmw_config.h
 *
 *   @brief
 *      This is the header file that describes configurations for the Millimeter
 *   Wave Demo.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2019 Texas Instruments, Inc.
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
#ifndef RANGING_CONFIG_H
#define RANGING_CONFIG_H

/* MMWAVE library Include Files */
#include <inc/ranging_dpc.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/common/sys_common.h>

/* Data path Include Files */
//#include <inc/objdetrangehwa.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  Millimeter Wave Demo Gui Monitor Selection
 *
 * @details
 *  The structure contains the selection for what information is placed to
 *  the output packet, and sent out to GUI. Unless otherwise specified,
 *  if the flag is set to 1, information
 *  is sent out. If the flag is set to 0, information is not sent out.
 *
 */
typedef struct Ranging_GuiMonSel_t
{

    /*! @brief   Send noise floor profile */
    uint8_t        noiseProfile;

    /*! @brief   Send stats */
    uint8_t        statsInfo;
} Ranging_GuiMonSel;

/**
 * @brief
 *  LVDS streaming configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  for the LVDS streaming.
 */
typedef struct Ranging_LvdsStreamCfg_t
{
    /**
     * @brief  HSI Header enabled/disabled flag. Only applicable for HW streaming.
     *         Will be ignored for SW streaming which will always have HSI header.
     */
    bool        isHeaderEnabled;

    /*! HW STREAMING DISABLED */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED   0

    /*! ADC */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_ADC        1

    /*! CP_ADC_CQ */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_CP_ADC_CQ  4

    /*! HW streaming data format:
        0-HW STREAMING DISABLED
        1-ADC
        2-Reserved
        3-Reserved
        4-CP_ADC_CQ
    */
    uint8_t     dataFmt;

    /**
     * @brief  SW enabled/disabled flag
     */
    bool        isSwEnabled;
}Ranging_LvdsStreamCfg;

/**
 * @brief
 *  Millimeter Wave Demo Platform Configuration.
 *
 * @details
 *  The structure is used to hold all the relevant configuration for
 *  the Platform.
 */
typedef struct Ranging_platformCfg_t
{
    /*! @brief   GPIO index for sensor status */
    uint32_t            SensorStatusGPIO;
    
    /*! @brief   CPU Clock Frequency. */
    uint32_t            sysClockFrequency;

    /*! @brief   UART Logging Baud Rate. */
    uint32_t            loggingBaudRate;

    /*! @brief   UART Command Baud Rate. */
    uint32_t            commandBaudRate;
} Ranging_platformCfg;

/**
 * @brief
 *  Millimeter Wave Demo configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  which is used to execute the Millimeter Wave Demo.
 */
typedef struct Ranging_Cfg_t
{
    /*! @brief   mmWave Control Configuration. */
    MMWave_CtrlCfg      ctrlCfg;

    /*! @brief   mmWave Open Configuration. */
    MMWave_OpenCfg      openCfg;

    /*! @brief   Platform specific configuration. */
    Ranging_platformCfg platformCfg;
} Ranging_Cfg;

/**
 * @brief
 *  Data path DPC common configuraiton.
 *
 * @details
 *  The structure is used to hold all the relevant configuration for
 *  DPC common configuration.
 */
typedef struct Ranging_DPC_ObjDet_CommonCfg_t
{

    /*! @brief pre start common config */
    DPC_Ranging_PreStartCommonCfg   preStartCommonCfg;
} Ranging_DPC_ObjDet_CommonCfg;

#ifdef __cplusplus
}
#endif

#endif /* MMW_CONFIG_H */
