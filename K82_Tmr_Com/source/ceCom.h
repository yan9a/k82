// Description: Ring buffered LPUART communication
// File: ceCom.h
// Author: Yan Naing Aye
// References:
//    http://cool-emerald.blogspot.com/2021/07/software-timers-and-ring-buffered-uart.html
//    http://cool-emerald.blogspot.com/2010/06/circular-buffered-uart-com-module-for.html

#ifndef	CE_COM_H
	#define	CE_COM_H
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "fsl_lpuart.h"
#include "pin_mux.h"
#include "clock_config.h"

#define CECOM0_ENABLE 0
#define CECOM1_ENABLE 0
#define CECOM2_ENABLE 0
#define CECOM3_ENABLE 0
#define CECOM4_ENABLE 1

#define CECOM_CLKSRC kCLOCK_Osc0ErClk
#define CECOM_CLK_FREQ CLOCK_GetFreq(kCLOCK_Osc0ErClk)

#if(CECOM0_ENABLE==1)
	#define COM_IRQHandler0 LPUART0_IRQHandler
	void ceComOnRx0(uint8_t b);
#endif
#if(CECOM1_ENABLE==1)
	#define COM_IRQHandler1 LPUART1_IRQHandler
	void ceComOnRx1(uint8_t b);
#endif
#if(CECOM2_ENABLE==1)
	#define COM_IRQHandler2 LPUART2_IRQHandler
	void ceComOnRx2(uint8_t b);
#endif
#if(CECOM3_ENABLE==1)
	#define COM_IRQHandler3 LPUART3_IRQHandler
	void ceComOnRx3(uint8_t b);
#endif
#if(CECOM4_ENABLE==1)
	#define COM_IRQHandler4 LPUART4_IRQHandler
	void ceComOnRx4(uint8_t b);
#endif

typedef void (*ceCom_RxCB_t)(uint8_t b);
//-----------------------------------------------------------------------------
void ceComInitialize();
void ceComInit(uint32_t uartNo, lpuart_config_t config, size_t bufSize, ceCom_RxCB_t cbFuncOnRx);
void ceComTask();
void ceComTaskRx(uint32_t uartNo);
void ceComTaskTx(uint32_t uartNo);
//-----------------------------------------------------------------------------
uint8_t ceComPutch(uint32_t uartNo, uint8_t c);
void ceComPrint(uint32_t uartNo, char *s);
void ceComWrite(uint32_t uartNo, uint8_t *s,size_t n);
void ceComPrintHex(uint32_t uartNo, uint8_t ch);
void ceComPrintHex32(uint32_t uartNo, uint32_t r);
void ceComPrintDec32(uint32_t uartNo, uint32_t r);
void ceComWUT(uint32_t uartNo);// wait until transmitted
//-----------------------------------------------------------------------------
#endif // CE_COM_H
