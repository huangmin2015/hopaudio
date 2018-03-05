/**
 *  \file   main.c
 *
 *  \brief  Example application main file. This application will toggle the led.
 *          The led toggling will be done inside an callback function, which
 *          will be called by Interrupt Service Routine. Interrupts are
 *          triggered manually and no external source is used to trigger
 *          interrupts.
 *
 */

/*
 * Copyright (C) 2014 - 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BARE_METAL
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Registry.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/MultiProc.h>
#endif

#include <stdio.h>

/* TI-RTOS Header files */
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>

#include <ti/drv/gpio/test/led_blink/src/GPIO_log.h>
#include <ti/drv/gpio/test/led_blink/src/GPIO_board.h>

#include <ti/board/board.h>

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/
#if defined(SOC_AM572x) || defined (SOC_AM571x)
#if defined (__TI_ARM_V7M4__)
#define DELAY_VALUE       (0x6FFFFU)
#else
#define DELAY_VALUE       (0x6FFFFFU)
#endif
#else
#define DELAY_VALUE       (0x6FFFFFU)
#endif

Clock_Handle hClock = NULL;

/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/* Delay function */
void AppDelay(unsigned int delayVal);

/* Callback function */
void AppGpioCallbackFxn(void);

/* clock callback function */
void clockFxn(UArg);

#if defined(IDK_AM572X) || defined(IDK_AM571X)
/* GPIO clock and pinmux configurations */
extern void AppGPIOInit(void);
#endif

#if defined(IDK_AM572X)
extern void GPIOApp_UpdateBoardInfo(void);
extern void GPIOAppUpdateConfig(uint32_t *gpioBaseAddr, uint32_t *gpioPin);
#endif

/*
 *  ======== Board_initI2C ========
 */
static void Board_initGPIO(void) {
	Board_initCfg boardCfg;

#if defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_K2E) || defined(SOC_K2L) || defined(SOC_K2G) || defined(SOC_C6678) || defined(SOC_C6657)
	GPIO_v0_HwAttrs gpio_cfg;

	/* Get the default SPI init configurations */
	GPIO_socGetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);

	/* Modify the default GPIO configurations if necessary */

	/* Set the default GPIO init configurations */
	GPIO_socSetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);

#if defined(SOC_K2G)
	/* Setup GPIO interrupt configurations */
	GPIO_socSetIntMux(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL, GPIO_MUX_SEL);
#endif
#endif

#if defined(EVM_K2E) || defined(EVM_C6678)
	boardCfg = BOARD_INIT_MODULE_CLOCK |
	BOARD_INIT_UART_STDIO;
#else
	boardCfg = BOARD_INIT_PINMUX_CONFIG |
	BOARD_INIT_MODULE_CLOCK |
	BOARD_INIT_UART_STDIO;
#endif
	//Board_init(boardCfg);

#if defined(IDK_AM572X)
	GPIOApp_UpdateBoardInfo();
#endif

}

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
volatile uint32_t gpio_intr_triggered = 0;
uint32_t gpioBaseAddr;
uint32_t gpioPin;

/*
 *  ======== test function ========
 */
#ifndef BARE_METAL
void gpio_test(UArg arg0, UArg arg1) {
#else
	void main()
	{
		Board_initGPIO();
#endif
	/* GPIO initialization */
	GPIO_init();

	/* Set the callback function */
	//GPIO_setCallback(USER_LED0, AppGpioCallbackFxn);
	/* Enable GPIO interrupt on the specific gpio pin */
	//GPIO_enableInt(USER_LED0);
	GPIO_write((USER_LED1), GPIO_PIN_VAL_HIGH);
	Clock_start(hClock);

	while (1) {
		/* Write high to gpio pin to control LED1 */
		GPIO_write((USER_LED1), GPIO_PIN_VAL_HIGH);
		//AppDelay(DELAY_VALUE);
		Task_sleep(1000);
		GPIO_write((USER_LED1), GPIO_PIN_VAL_LOW);
		//AppDelay(DELAY_VALUE);
		Task_sleep(1000);
	}

#if defined(SOC_K2L) || defined(SOC_C6678) || defined(SOC_C6657)
	/* No GPIO pin directly connected to user LED's on K2L/K2G/C6678/C6657 EVM, just trigger interrupt once */
	GPIO_log("\n GPIO Test Application \n");

	GPIO_toggle(USER_LED0);
	while (!gpio_intr_triggered);

	GPIO_log("\n GPIO Test Passed \n");
#else
	GPIO_log("\n GPIO Led Blink Application \n");

	while (1) {
#if defined(SOC_AM572x) || defined(SOC_AM571x)|| defined(SOC_AM335x)  || defined(SOC_AM437x)

#if defined (IDK_AM572X)
		/* Update GPIO info based on the board */
		GPIOAppUpdateConfig(&gpioBaseAddr, &gpioPin);
#else   //modify by hum
		gpioBaseAddr = 0;//GPIO_BASE_ADDR;
		gpioPin = 0;//GPIO_LED_PIN;
#endif
		/* Trigger interrupt */
		//GPIOTriggerPinInt(gpioBaseAddr, 0, gpioPin);
#endif
#if defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_K2E) || defined(SOC_K2G)
		GPIO_toggle(USER_LED0);
#endif
		AppDelay(DELAY_VALUE);
	}
#endif
}

//BWC, declare ipc_main as external
extern Int ipc_main();
extern Void audio_main(Void);
//BWC declare as external, will call directly instead of having BIOS call it, change needed in cfg file too
extern Void IpcMgr_ipcStartup(Void);
extern Void Audio_echo_Task();
#ifndef BARE_METAL
/*
 *  ======== main ========
 */
int main(void) {
	Task_Handle task;
	Clock_Params clockParams;
	Error_Block eb;
	int callIpcStartup = 1;
	Log_print0(Diags_ENTRY, " 20180215 main inti by hum...:");
	/* Call board init functions */
	//Board_initGPIO();
#if defined(IDK_AM572X) || defined(IDK_AM571X)
	AppGPIOInit();
#endif

	Error_init(&eb);
	//task = Task_create(gpio_test, NULL, &eb);
#if 1
	task = Task_create(Audio_echo_Task, NULL, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}
	Log_print0(Diags_ENTRY, " Task_create Audio_echo_Task...:");
#endif
	//BWC, call the ipc main function
	audio_main();
#if 1  //close ipc
	ipc_main();

	if (callIpcStartup) {
		IpcMgr_ipcStartup();
	}
    Log_print0(Diags_ENTRY, "IpcMgr_ipcStartup...:");
#endif
	/* Start BIOS */
	BIOS_start();
	return (0);
}
#endif

/*
 *  ======== AppDelay ========
 */
void AppDelay(unsigned int delayVal) {
	while (delayVal) {
		delayVal--;
	}
}

/*
 *  ======== Callback function ========
 */
void AppGpioCallbackFxn(void) {
	/* Toggle LED1 */
	GPIO_toggle(USER_LED1);
	AppDelay(DELAY_VALUE);
	gpio_intr_triggered = 1;
}

void clockFxn(UArg arg) {

	asm(" NOP");
	GPIO_toggle(USER_LED1);
}

