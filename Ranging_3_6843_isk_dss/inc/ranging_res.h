/**
 *   @file  ranging_res.h
 *
 *   @brief
 *      Defines partitioning of hardware resources (EDMA etc) among the
 *      DPCs and other components in the millimeter wave demo.
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
#ifndef RANGING_RES_H
#define RANGING_RES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/edma/edma.h>
#include <ti/common/sys_common.h>

#define EDMA_INSTANCE_0 0
#define EDMA_INSTANCE_1 1

/*******************************************************************************
 * Resources for Object Detection DPC, currently the only DPC and edma
 * resource user in the demo.
 *******************************************************************************/
/* EDMA instance used*/
#define DPC_RANGING_DSP_EDMA_INSTANCE                                      EDMA_INSTANCE_0
#define DPC_RANGING_DSP_EDMA_SHADOW_BASE                                   EDMA_NUM_DMA_CHANNELS

/* Range DPU */
#define DPC_RANGING_DSP_DPU_EDMA_INST_ID                         DPC_RANGING_DSP_EDMA_INSTANCE
#define DPC_RANGING_DSP_DPU_EDMAIN_PING_CH                       EDMA_TPCC0_REQ_FREE_0
#define DPC_RANGING_DSP_DPU_EDMAIN_PING_SHADOW                   (DPC_RANGING_DSP_EDMA_SHADOW_BASE + 0)
#define DPC_RANGING_DSP_DPU_EDMAIN_PING_EVENT_QUE                0
#define DPC_RANGING_DSP_DPU_EDMAIN_PONG_CH                       EDMA_TPCC0_REQ_FREE_1
#define DPC_RANGING_DSP_DPU_EDMAIN_PONG_SHADOW                   (DPC_RANGING_DSP_EDMA_SHADOW_BASE + 1)
#define DPC_RANGING_DSP_DPU_EDMAIN_PONG_EVENT_QUE                0

#define DPC_RANGING_DSP_DPU_EDMAOUT_PING_CH                      EDMA_TPCC0_REQ_FREE_2
#define DPC_RANGING_DSP_DPU_EDMAOUT_PING_SHADOW                  (DPC_RANGING_DSP_EDMA_SHADOW_BASE + 2)
#define DPC_RANGING_DSP_DPU_EDMAOUT_PING_EVENT_QUE               0
#define DPC_RANGING_DSP_DPU_EDMAOUT_PONG_CH                      EDMA_TPCC0_REQ_FREE_3
#define DPC_RANGING_DSP_DPU_EDMAOUT_PONG_SHADOW                  (DPC_RANGING_DSP_EDMA_SHADOW_BASE + 3)
#define DPC_RANGING_DSP_DPU_EDMAOUT_PONG_EVENT_QUE               0

/*************************LVDS streaming EDMA resources*******************************/
/*EDMA instance used*/
#define MMW_LVDS_STREAM_EDMA_INSTANCE            EDMA_INSTANCE_1
#define MMW_LVDS_STREAM_EDMA_SHADOW_BASE         EDMA_NUM_DMA_CHANNELS

/* CBUFF EDMA trigger channels */
#define MMW_LVDS_STREAM_CBUFF_EDMA_CH_0          EDMA_TPCC1_REQ_CBUFF_0
#define MMW_LVDS_STREAM_CBUFF_EDMA_CH_1          EDMA_TPCC1_REQ_CBUFF_1

/* HW Session*/
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_0     EDMA_TPCC1_REQ_FREE_0
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_1     EDMA_TPCC1_REQ_FREE_1
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_2     EDMA_TPCC1_REQ_FREE_2
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_3     EDMA_TPCC1_REQ_FREE_3
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_4     EDMA_TPCC1_REQ_FREE_4
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_5     EDMA_TPCC1_REQ_FREE_5
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_6     EDMA_TPCC1_REQ_FREE_6
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_7     EDMA_TPCC1_REQ_FREE_7
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_8     EDMA_TPCC1_REQ_FREE_8
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_9     EDMA_TPCC1_REQ_FREE_9
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_10    EDMA_TPCC1_REQ_FREE_10

/* SW Session*/
#define MMW_LVDS_STREAM_SW_SESSION_EDMA_CH_0     EDMA_TPCC1_REQ_FREE_11
#define MMW_LVDS_STREAM_SW_SESSION_EDMA_CH_1     EDMA_TPCC1_REQ_FREE_12
#define MMW_LVDS_STREAM_SW_SESSION_EDMA_CH_2     EDMA_TPCC1_REQ_FREE_13

/*shadow*/
/*shadow CBUFF trigger channels*/
#define MMW_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_0   (MMW_LVDS_STREAM_EDMA_SHADOW_BASE + 0U)
#define MMW_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_1   (MMW_LVDS_STREAM_EDMA_SHADOW_BASE + 1U)

/* HW Session*/
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_0   (MMW_LVDS_STREAM_EDMA_SHADOW_BASE + 2U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_1   (MMW_LVDS_STREAM_EDMA_SHADOW_BASE + 3U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_2   (MMW_LVDS_STREAM_EDMA_SHADOW_BASE + 4U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_3   (MMW_LVDS_STREAM_EDMA_SHADOW_BASE + 5U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_4   (MMW_LVDS_STREAM_EDMA_SHADOW_BASE + 6U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_5   (MMW_LVDS_STREAM_EDMA_SHADOW_BASE + 7U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_6   (MMW_LVDS_STREAM_EDMA_SHADOW_BASE + 8U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_7   (MMW_LVDS_STREAM_EDMA_SHADOW_BASE + 9U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_8   (MMW_LVDS_STREAM_EDMA_SHADOW_BASE + 10U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_9   (MMW_LVDS_STREAM_EDMA_SHADOW_BASE + 11U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_10  (MMW_LVDS_STREAM_EDMA_SHADOW_BASE + 12U)

/* SW Session*/
#define MMW_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_0   (EDMA_NUM_DMA_CHANNELS + 13U)
#define MMW_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_1   (EDMA_NUM_DMA_CHANNELS + 14U)
#define MMW_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_2   (EDMA_NUM_DMA_CHANNELS + 15U)

/*************************LVDS streaming EDMA resources END*******************************/

#ifdef __cplusplus
}
#endif

#endif /*RANGING_RES_H */

