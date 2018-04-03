/*
 * Audio_evmInit.h
 *
 * This file contains Application programming interface for the Audio application
 * related EVM (platform) specifc initialization routines
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 *
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
 *
*/

/**
 * \file   Audio_evmInit.h
 *
 * \brief  This file contains the board level functions of Audio driver.
 *
 *  (C) Copyright 2009, Texas Instruments, Inc
 *
 */

#ifndef _MCASP_CFG_H_
#define _MCASP_CFG_H_

#include <xdc/std.h>
#include <string.h>
#include <xdc/runtime/Log.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/io/GIO.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>

#include <mcasp_drv.h>
#include <ti/sysbios/io/IOM.h>
#include <Aic31.h>

//#include <Audio.h>
#include <ti/sdo/edma3/drv/edma3_drv.h>
#include <ti/csl/csl_edma3.h>
#include <ti/csl/soc.h>
#include <ti/csl/csl_bootcfg.h>
#ifdef __cplusplus
extern "C"
{
#endif


#define EDMACC_NUM 0
//I2C CFG
//#define SOC_I2C_1_REGS CSL_DSP_I2C3_REGS
#define I2C_MCASP_INSTANCE 2


#define SOC_I2C_1_REGS CSL_DSP_I2C1_REGS
#define I2C_1_INSTANCE   0
#define SOC_I2C_4_REGS CSL_DSP_I2C4_REGS
#define I2C_4_INSTANCE   3




#define AIC31_NUM_INSTANCES   1
#define AIC31_INST0_ADDRESS   0x4a//0x4a//0x1a
//I2C slave addr
#define AIC31_I2C_ADDR      0x1b
#define PW8960_I2C_ADDR     0x1a

#define PCM1864_I2C_ADDR    0x4a
#define PCM1864_2_I2C_ADDR    0x4b
#define PCM1864_I2C3_1_ADDR    0x4a
#define PCM1864_I2C3_2_ADDR    0x4b
#define PCM1864_I2C4_1_ADDR    0x4a
#define PCM1864_I2C4_2_ADDR    0x4b



//MCASP CFG
//#define MCASP_RX_DMA_CH  CSL_EDMA3_CHA_MCASP2_RX
//#define MCASP_TX_DMA_CH  CSL_EDMA3_CHA_MCASP2_TX
//#define MCASP_NUM 2
#define MCASP_MIC_ARRAY_NUM     0
#define MCASP_PLAYBACK_NUM      1
#define MCASP_RX_DMA_MIC_ARRAY  CSL_EDMA3_CHA_MCASP0_RX
#define MCASP_TX_DMA_PLAYBACK   CSL_EDMA3_CHA_MCASP1_TX




extern void GblErr(int arg);
extern Mcasp_HwSetupData mcaspRcvSetup;
extern Mcasp_HwSetupData mcaspXmtSetup;
extern Mcasp_ChanParams  mcasp_chanparam[];
extern ICodec_ChannelConfig AIC31_config;

/** Number of serializers configured for record */

#ifdef DEVICE_LOOPBACK
#define RX_NUM_SERIALIZER       (2u)
#define TX_NUM_SERIALIZER       (2u)
#else
#define RX_NUM_SERIALIZER       (8u)
#define TX_NUM_SERIALIZER       (2u)
#define MIC_TX_NUM_SERIALIZER       (1u)
#endif

#ifdef __cplusplus
}
#endif

#endif /* _AUDIO_EVMINIT_H_ */
/* ========================================================================== */
/*                              END OF FILE                                   */
/* ========================================================================== */
