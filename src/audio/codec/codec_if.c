/**
 * \file  codecif.c
 *
 * \brief Functions to configure the codec trough i2c or other interfaces.
 *        Currently only one interface type is allowed. If another interface to be
 *        used, this need enhancement.
 *
 *  This file contains the implementation of the AIC31 audio codec driver for
 *  DSP BIOS operating system.
 *
 *  (C) Copyright 2012, Texas Instruments, Inc
 */

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

/* ========================================================================== */
/*                            INCLUDE FILES                                   */
/* ========================================================================== */
//#include "soc_C6748.h"
//#include "interrupt.h"
//#include "hw_syscfg0_C6748.h"
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>
#include <codec_if.h>
#include <mcasp_cfg.h>
#include "MCASP_log.h"
/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/

#define     I2C_SUCCESS          1
#define     I2C_FAILURE         -1

#define     I2C_WRITE           100
#define     I2C_READ            101

#define     I2C_DELAY_SMALL     0x0200
#define     I2C_DELAY_BIG       0xFF00

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/

static unsigned int I2CCodecSendBlocking(unsigned int baseAddr,unsigned char slaveAddr, unsigned int dataCnt);
static unsigned int I2CCodecRcvBlocking(unsigned int baseAddr,unsigned char slaveAddr, unsigned int dataCnt);
/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
volatile unsigned int dataIdx = 0;
volatile unsigned int txCompFlag = 1;
volatile unsigned char slaveData[3];
volatile unsigned int dataMax;
I2C_Handle I2C_handle_glob = NULL;
I2C_Handle I2c1_handle=NULL;
I2C_Handle I2c4_handle=NULL;
unsigned int I2C_slaveAddr = 0;

static void I2CCodecuDelay(unsigned int delay)
{
    volatile unsigned int del = delay;
    while(del--) {}
}
/******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/

/**
 * \brief   Initializes the I2C interface for a codec
 *
 * \param   baseAddr      Base Address of the I2C Module Registers which
 *                        is used for the codec
 *          intCh         Channel Number where the I2C ISR to be registered
 *          slaveAddr     Slave Address of the codec
 *
 * Note: This API enables the system interrupt for the given I2C module only.
 *       It does not do any pin multiplexing or global interrupt enabling.
 *       This shall be called only after AINTC initialization.
 *
 * \return  None.
 *
 **/
void I2CCodecIfInit1(unsigned int baseAddr, unsigned int intCh,
                    unsigned int slaveAddr)
{
    I2C_Params i2cParams;
	I2C_HwAttrs mcasp_i2c_cfg;
    I2C_slaveAddr =(uint32_t)slaveAddr;
	I2C_init();

    I2C_Params_init(&i2cParams);
    i2cParams.transferMode = I2C_MODE_BLOCKING;

    I2C_socGetInitCfg(I2C_MCASP_INSTANCE, &mcasp_i2c_cfg);

    /* Modify the default I2C configurations if necessary */
    mcasp_i2c_cfg.enableIntr=false; 
    /* Set the default I2C init configurations */
    I2C_socSetInitCfg(I2C_MCASP_INSTANCE, &mcasp_i2c_cfg);
    I2C_handle_glob = I2C_open(I2C_MCASP_INSTANCE, &i2cParams);
}
void I2CCodecIfInit_for_MicArray(unsigned int baseAddr, unsigned int i2c_instance,unsigned int intCh)
{
    I2C_Params i2cParams;
	I2C_HwAttrs mcasp_i2c_cfg;
	I2C_Handle I2C_handle;
	//I2C_init();

    I2C_Params_init(&i2cParams);
    i2cParams.transferMode = I2C_MODE_BLOCKING;

    I2C_socGetInitCfg(i2c_instance, &mcasp_i2c_cfg);

    /* Modify the default I2C configurations if necessary */
    mcasp_i2c_cfg.enableIntr=false; 
    /* Set the default I2C init configurations */
    I2C_socSetInitCfg(i2c_instance, &mcasp_i2c_cfg);
    if(baseAddr==SOC_I2C_1_REGS)
        I2c1_handle = I2C_open(i2c_instance, &i2cParams);
    else
        I2c4_handle = I2C_open(i2c_instance, &i2cParams);

}

/*
** Function to send data through i2c
*/
static unsigned int I2CCodecSendBlocking(unsigned int baseAddr,unsigned char slaveAddr, unsigned int dataCnt)
{
    unsigned int status = I2C_SUCCESS;
    I2C_Transaction i2cTransaction;
    I2C_transactionInit(&i2cTransaction);
    i2cTransaction.slaveAddress = slaveAddr;
    i2cTransaction.writeBuf = (uint8_t *)&slaveData[0];
    i2cTransaction.writeCount = dataCnt;
    i2cTransaction.readBuf = (uint8_t *)NULL;
    i2cTransaction.readCount = 0;
    if(baseAddr==SOC_I2C_1_REGS)
        status = I2C_transfer(I2c1_handle, &i2cTransaction);
    else
        status = I2C_transfer(I2c4_handle, &i2cTransaction);

    I2CCodecuDelay(0xFF00);
    return (status);
}

/*
** Function to receive data from the Codec through I2C bus
*/
static unsigned int I2CCodecRcvBlocking(unsigned int baseAddr,unsigned char slaveAddr, unsigned int dataCnt)
{
    unsigned int status = I2C_SUCCESS;
    unsigned char writebuf[1];
    I2C_Transaction i2cTransaction;

    writebuf[0]=slaveData[0];
    I2C_transactionInit(&i2cTransaction);
    i2cTransaction.slaveAddress = slaveAddr;
    i2cTransaction.writeBuf = (uint8_t *)&writebuf[0];
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = (uint8_t *)&slaveData[0];
    i2cTransaction.readCount = dataCnt;
    if(baseAddr==SOC_I2C_1_REGS)
            status = I2C_transfer(I2c1_handle, &i2cTransaction);
        else
            status = I2C_transfer(I2c4_handle, &i2cTransaction);

    return (status);

}


/*
** Writes a codec register with the given data value
*/
void CodecRegWrite(unsigned int baseAddr, unsigned char regAddr,
                   unsigned char regData)
{
    unsigned int retVal = I2C_SUCCESS;

#ifdef CODEC_INTERFACE_I2C
    MCASP_log ("Debug: baseadr: %02x AIC31_addr:%02x  AIC31_data: %02x\n", baseAddr,regAddr,regData);

    /* Send the register address and data */
    slaveData[0] = regAddr;
    slaveData[1] = regData;

    retVal = I2CCodecSendBlocking(baseAddr,AIC31_I2C_ADDR, 2);

    if (I2C_SUCCESS != retVal)
    {
    }
#endif
}
void CodecRegWrite_wm8960(unsigned int baseAddr, unsigned char slaveAddr,unsigned char regAddr,
                   unsigned int regData)
{
    unsigned int retVal = I2C_SUCCESS;
#ifdef CODEC_INTERFACE_I2C
    /* Send the register address and data */
    unsigned int daWord;
    daWord=0;
    daWord=(regAddr<<9) & 0xfe00;
    daWord|=(regData & 0x1ff);
    slaveData[0] = (daWord>>8);
    slaveData[1] = daWord & 0xff;

    retVal = I2CCodecSendBlocking(baseAddr,slaveAddr, 2);

    if (I2C_SUCCESS != retVal)
    {
    }
#endif
}
void CodecRegWrite_PCM1864(unsigned int baseAddr,unsigned char slaveAddr, unsigned char regAddr,
                   unsigned char regData)
{
    unsigned int retVal = I2C_SUCCESS;
#ifdef CODEC_INTERFACE_I2C

    /* Send the register address and data */
    slaveData[0] = regAddr;
    slaveData[1] = regData;

    retVal = I2CCodecSendBlocking(baseAddr,slaveAddr, 2);

    if (I2C_SUCCESS != retVal)
    {
    }
#endif
}
/*
** Reads a codec register contents
*/
unsigned char CodecRegRead(unsigned int baseAddr,unsigned char slaveAddr, unsigned char regAddr)
{
    unsigned int retVal = I2C_SUCCESS;
#ifdef CODEC_INTERFACE_I2C
    slaveData[0] = regAddr;
     /* Receive the register contents in slaveData */
    retVal = I2CCodecRcvBlocking(baseAddr,slaveAddr, 1);

    if (I2C_SUCCESS != retVal)
    {
    }
#endif
    return (slaveData[0]);
}


/*
** Sets codec register bit specified in the bit mask
*/
void CodecRegBitSet(unsigned int baseAddr,unsigned char slaveAddr,unsigned char regAddr,
                    unsigned char bitMask)
{
    unsigned int retVal = 0;
#ifdef CODEC_INTERFACE_I2C

    /* Send the register address */
    slaveData[0] = regAddr;
    /* Receive the register contents in slaveData */
    retVal = I2CCodecRcvBlocking(baseAddr,slaveAddr, 2);
    if (I2C_SUCCESS != retVal)
    {
         //printf("\r\nI2C Read Failed\n");
         //retVal = retVal;
    }
    slaveData[1] =  slaveData[0] | bitMask;
    slaveData[0] = regAddr;

    retVal = I2CCodecSendBlocking(baseAddr,slaveAddr, 2);
    if (I2C_SUCCESS != retVal)
    {
    }
#endif
}

/*
** Clears codec register bits specified in the bit mask
*/
void CodecRegBitClr(unsigned int baseAddr,unsigned char regAddr,
                    unsigned char bitMask)
{
    unsigned int retVal = 0;

#ifdef CODEC_INTERFACE_I2C

    /* Send the register address */
    slaveData[0] = regAddr;
    /* Receive the register contents in slaveData */
    retVal = I2CCodecRcvBlocking(baseAddr,AIC31_I2C_ADDR, 1);
    {
    }
    slaveData[1] =  slaveData[0] & ~bitMask;
    slaveData[0] = regAddr;

    retVal = I2CCodecSendBlocking(baseAddr,AIC31_I2C_ADDR, 2);
    if (I2C_SUCCESS != retVal)
    {
    }
#endif
}

/***************************** End Of File ***********************************/
