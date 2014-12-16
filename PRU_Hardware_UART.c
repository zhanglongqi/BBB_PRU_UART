/*
 * Copyright (c) 2014, Texas Instruments, Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE. */

#include <stdint.h>
#include <pru_uart.h>

#include "hw_types.h"
#include "soc_AM335x.h"
#include "hw_cm_wkup.h"
#include "hw_cm_per.h"
#include "hw_control_AM335x.h"

#include "uart_irda_cir.h"

void UARTPinMuxSetup(unsigned int instanceNum);
void UART0ModuleClkConfig(void);
/* Mapping Constant table register to variable */
volatile far pruUart CT_UART __attribute__((cregister("PRU_UART0", near), peripheral));

/* Commented out to prevent internal Loopback */
//#define HWLOOPBACK
/* If SWLOOPBACK is defined, this flag tells the main loop to copy rxBuf to txBuf */
#ifdef SWLOOPBACK
uint8_t lpbkFlag;
#endif

/* The FIFO size on the PRU UART is 16 bytes; however, we are (arbitrarily)
 * only going to send 8 at a time */
#define FIFO_SIZE	16
#define MAX_CHARS	8

/* Buffer type */
typedef struct {
	uint8_t data[FIFO_SIZE];
} uartBuffer;

/* This hostBuffer structure allows the PRU to know the host is filling
 * the data memory starting at address 0 */
#pragma LOCATION(hostBuffer, 0)
struct {
	uint8_t msg;
	uartBuffer data[FIFO_SIZE];
} hostBuffer;

/* This rxBuf structure is used by the receive data */
#pragma LOCATION(rxBuf, 0x100)
uartBuffer rxBuf;

/* This txBuf structure is defined to store send data */
#pragma LOCATION(txBuf, 0x200)
uartBuffer txBuf;

//******************************************************************************
//    Print Message Out
//      This function take in a string literal of any size and then fill the
//      TX FIFO when it's empty and waits until there is info in the RX FIFO
//      before returning.
//******************************************************************************
void PrintMessageOut(volatile char* Message, uint8_t MsgSize) {
	uint8_t cnt, index = 0;

	while (1) {
		cnt = 0;

		/* Wait until the TX FIFO and the TX SR are completely empty */
		while (!CT_UART.LSR_bit.TEMT)
			;

		while (index < MsgSize && cnt < MAX_CHARS) {
			CT_UART.THR = Message[index];
			index++;
			cnt++;
		}
		if (index == MsgSize)
			break;

	}

	/* Wait until the TX FIFO and the TX SR are completely empty */
	while (!CT_UART.LSR_bit.TEMT)
		;

	/* Clear the TX FIFO */
	CT_UART.FCR_bit.TXCLR = 0x1;
	CT_UART.FCR_bit.TXCLR = 0x0;

}

//******************************************************************************
//    IEP Timer Config
//      This function waits until there is info in the RX FIFO and then returns
//      the first character entered.
//******************************************************************************
char ReadMessageIn() {
	while (!CT_UART.LSR_bit.DR)
		;

	return CT_UART.RBR_bit.DATA;
}

void main() {
	uint32_t i;
	volatile uint32_t not_done = 1;

	char rxBuffer[5];

	char GreetingMessage[] =
			"Hello you are in the PRU UART demo test please enter 5 characters  ";

	char ReplyMessage[] = "you typed: ";

	char NewLine[] = { 0x0A, 0x0D, 0x00 };


	/* enable ocp wide access */
	__asm__ __volatile__
            (
            " LBCO &r0, C4, 4, 4 \n"
            " CLR r0, r0, 4 \n"
            " SBCO &r0, C4, 4, 4 \n"
            );
	/*** INITIALIZATION ***/
	UARTPinMuxSetup(0);

	//enable the UART0 Clock module
	UART0ModuleClkConfig();

	/* Set up UART to function at 115200 baud - DLL divisor is 104 at 16x oversample
	 * 192MHz / 104 / 16 = ~115200 */
	CT_UART.DLL = 104;
	CT_UART.DLH = 0;
	CT_UART.MDR_bit.OSM_SEL = 0x0;

	/* Enable Interrupts in UART module. This allows the main thread to poll for
	 * Receive Data Available and Transmit Holding Register Empty */
	CT_UART.IER = 0x7;

	/* If FIFOs are to be used, select desired trigger level and enable
	 * FIFOs by writing to FCR. FIFOEN bit in FCR must be set first before
	 * other bits are configured */
	/* Enable FIFOs for now at 1-byte, and flush them */
	CT_UART.FCR = (0x80) | (0x4) | (0x2) | (0x01); // 8-byte RX FIFO trigger

	/* Choose desired protocol settings by writing to LCR */
	/* 8-bit word, 1 stop bit, no parity, no break control and no divisor latch */
	CT_UART.LCR = 3;

	/* If flow control is desired write appropriate values to MCR. */
	/* No flow control for now, but enable loopback for test */
#ifdef HWLOOPBACK
	CT_UART.MCR = 0x10;
#elif SWLOOPBACK
	CT_UART.MCR = 0x00;
#endif

	/* Choose desired response to emulation suspend events by configuring
	 * FREE bit and enable UART by setting UTRST and URRST in PWREMU_MGMT */
	/* Allow UART to run free, enable UART TX/RX */
	CT_UART.PWREMU_MGMT_bit.FREE = 0x1;
	CT_UART.PWREMU_MGMT_bit.URRST = 0x1;
	CT_UART.PWREMU_MGMT_bit.UTRST = 0x1;

	/* Turn off RTS and CTS functionality */
	CT_UART.MCR_bit.AFE = 0x0;
	CT_UART.MCR_bit.RTS = 0x0;

	/*** END INITIALIZATION ***/

	/* Print out greeting message */
	PrintMessageOut(GreetingMessage, sizeof(GreetingMessage));

	/* Print out new line */
	PrintMessageOut(NewLine, 2);

	/* Read in 5 characters from user, then echo them back out */
	for (i = 0; i < 5; i++) {
		rxBuffer[i] = ReadMessageIn();
	}

	PrintMessageOut(ReplyMessage, sizeof(ReplyMessage));

	/* Print out new line */
	PrintMessageOut(NewLine, 2);

	PrintMessageOut(rxBuffer, sizeof(rxBuffer));

	/* Print out new line */
	PrintMessageOut(NewLine, 2);

	/*** DONE SENDING DATA ***/
	/* Disable UART before halting */
	CT_UART.PWREMU_MGMT = 0x0;

	/* Halt PRU core */
	__halt();
}

/**
 * \brief   This function selects the UART pins for use. The UART pins
 *          are multiplexed with pins of other peripherals in the SoC
 *
 * \param   instanceNum       The instance number of the UART to be used.
 *
 * \return  None.
 *
 * \note    This pin multiplexing depends on the profile in which the EVM
 *          is configured.
 */
void UARTPinMuxSetup(unsigned int instanceNum) {
	if (0 == instanceNum) {
		/* RXD */
		HWREG(SOC_CONTROL_REGS + CONTROL_CONF_UART_RXD(0)) =
				(CONTROL_CONF_UART0_RXD_CONF_UART0_RXD_PUTYPESEL |
				CONTROL_CONF_UART0_RXD_CONF_UART0_RXD_RXACTIVE);

		/* TXD */
		HWREG(SOC_CONTROL_REGS + CONTROL_CONF_UART_TXD(0)) =
		CONTROL_CONF_UART0_TXD_CONF_UART0_TXD_PUTYPESEL;
	}
}
/*
 ** This function enables the system L3 and system L4_WKUP clocks.
 ** This also enables the clocks for UART0 instance.
 */

void UART0ModuleClkConfig(void) {
	/* Configuring L3 Interface Clocks. */

	/* Writing to MODULEMODE field of CM_PER_L3_CLKCTRL register. */
	HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) |=
	CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

	/* Waiting for MODULEMODE field to reflect the written value. */
	while (CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE
			!= (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) &
			CM_PER_L3_CLKCTRL_MODULEMODE))
		;

	/* Writing to MODULEMODE field of CM_PER_L3_INSTR_CLKCTRL register. */
	HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) |=
	CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;

	/* Waiting for MODULEMODE field to reflect the written value. */
	while (CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE
			!= (HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) &
			CM_PER_L3_INSTR_CLKCTRL_MODULEMODE))
		;

	/* Writing to CLKTRCTRL field of CM_PER_L3_CLKSTCTRL register. */
	HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) |=
	CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

	/* Waiting for CLKTRCTRL field to reflect the written value. */
	while (CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP
			!= (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) &
			CM_PER_L3_CLKSTCTRL_CLKTRCTRL))
		;

	/* Writing to CLKTRCTRL field of CM_PER_OCPWP_L3_CLKSTCTRL register. */
	HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) |=
	CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

	/*Waiting for CLKTRCTRL field to reflect the written value. */
	while (CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP
			!= (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
			CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL))
		;

	/* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
	HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) |=
	CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

	/*Waiting for CLKTRCTRL field to reflect the written value. */
	while (CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP
			!= (HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) &
			CM_PER_L3S_CLKSTCTRL_CLKTRCTRL))
		;

	/* Checking fields for necessary values.  */

	/* Waiting for IDLEST field in CM_PER_L3_CLKCTRL register to be set to 0x0. */
	while ((CM_PER_L3_CLKCTRL_IDLEST_FUNC << CM_PER_L3_CLKCTRL_IDLEST_SHIFT)
			!= (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) &
			CM_PER_L3_CLKCTRL_IDLEST))
		;

	/*
	 ** Waiting for IDLEST field in CM_PER_L3_INSTR_CLKCTRL register to attain the
	 ** desired value.
	 */
	while ((CM_PER_L3_INSTR_CLKCTRL_IDLEST_FUNC <<
	CM_PER_L3_INSTR_CLKCTRL_IDLEST_SHIFT)
			!= (HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) &
			CM_PER_L3_INSTR_CLKCTRL_IDLEST))
		;

	/*
	 ** Waiting for CLKACTIVITY_L3_GCLK field in CM_PER_L3_CLKSTCTRL register to
	 ** attain the desired value.
	 */
	while (CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK
			!= (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) &
			CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK))
		;

	/*
	 ** Waiting for CLKACTIVITY_OCPWP_L3_GCLK field in CM_PER_OCPWP_L3_CLKSTCTRL
	 ** register to attain the desired value.
	 */
	while (CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK
			!= (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
			CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK))
		;

	/*
	 ** Waiting for CLKACTIVITY_L3S_GCLK field in CM_PER_L3S_CLKSTCTRL register
	 ** to attain the desired value.
	 */
	while (CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK
			!= (HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) &
			CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK))
		;

	/* Configuring registers related to Wake-Up region. */

	/* Writing to MODULEMODE field of CM_WKUP_CONTROL_CLKCTRL register. */
	HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) |=
	CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE;

	/* Waiting for MODULEMODE field to reflect the written value. */
	while (CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE
			!= (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
			CM_WKUP_CONTROL_CLKCTRL_MODULEMODE))
		;

	/* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
	HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) |=
	CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

	/*Waiting for CLKTRCTRL field to reflect the written value. */
	while (CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP
			!= (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
			CM_WKUP_CLKSTCTRL_CLKTRCTRL))
		;

	/* Writing to CLKTRCTRL field of CM_L3_AON_CLKSTCTRL register. */
	HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) |=
	CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

	/*Waiting for CLKTRCTRL field to reflect the written value. */
	while (CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP
			!= (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
			CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL))
		;

	/* Writing to MODULEMODE field of CM_WKUP_UART0_CLKCTRL register. */
	HWREG(SOC_CM_WKUP_REGS + CM_WKUP_UART0_CLKCTRL) |=
	CM_WKUP_UART0_CLKCTRL_MODULEMODE_ENABLE;

	/* Waiting for MODULEMODE field to reflect the written value. */
	while (CM_WKUP_UART0_CLKCTRL_MODULEMODE_ENABLE
			!= (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_UART0_CLKCTRL) &
			CM_WKUP_UART0_CLKCTRL_MODULEMODE))
		;

	/* Verifying if the other bits are set to required settings. */

	/*
	 ** Waiting for IDLEST field in CM_WKUP_CONTROL_CLKCTRL register to attain
	 ** desired value.
	 */
	while ((CM_WKUP_CONTROL_CLKCTRL_IDLEST_FUNC <<
	CM_WKUP_CONTROL_CLKCTRL_IDLEST_SHIFT)
			!= (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
			CM_WKUP_CONTROL_CLKCTRL_IDLEST))
		;

	/*
	 ** Waiting for CLKACTIVITY_L3_AON_GCLK field in CM_L3_AON_CLKSTCTRL
	 ** register to attain desired value.
	 */
	while (CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK
			!= (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
			CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK))
		;

	/*
	 ** Waiting for IDLEST field in CM_WKUP_L4WKUP_CLKCTRL register to attain
	 ** desired value.
	 */
	while ((CM_WKUP_L4WKUP_CLKCTRL_IDLEST_FUNC <<
	CM_WKUP_L4WKUP_CLKCTRL_IDLEST_SHIFT)
			!= (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL) &
			CM_WKUP_L4WKUP_CLKCTRL_IDLEST))
		;

	/*
	 ** Waiting for CLKACTIVITY_L4_WKUP_GCLK field in CM_WKUP_CLKSTCTRL register
	 ** to attain desired value.
	 */
	while (CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK
			!= (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
			CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK))
		;

	/*
	 ** Waiting for CLKACTIVITY_L4_WKUP_AON_GCLK field in CM_L4_WKUP_AON_CLKSTCTRL
	 ** register to attain desired value.
	 */
	while (CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK
			!= (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL) &
			CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK))
		;

	/*
	 ** Waiting for CLKACTIVITY_UART0_GFCLK field in CM_WKUP_CLKSTCTRL
	 ** register to attain desired value.
	 */
	while (CM_WKUP_CLKSTCTRL_CLKACTIVITY_UART0_GFCLK
			!= (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
			CM_WKUP_CLKSTCTRL_CLKACTIVITY_UART0_GFCLK))
		;

	/*
	 ** Waiting for IDLEST field in CM_WKUP_UART0_CLKCTRL register to attain
	 ** desired value.
	 */
	while ((CM_WKUP_UART0_CLKCTRL_IDLEST_FUNC <<
	CM_WKUP_UART0_CLKCTRL_IDLEST_SHIFT)
			!= (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_UART0_CLKCTRL) &
			CM_WKUP_UART0_CLKCTRL_IDLEST))
		;
}

