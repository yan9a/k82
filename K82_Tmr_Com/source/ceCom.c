// Description: Ring buffered LPUART communication
// File: ceCom.h
// Author: Yan Naing Aye
// References:
//    http://cool-emerald.blogspot.com/2010/06/circular-buffered-uart-com-module-for.html

#include "ceCom.h"
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "ceRingBuf.h"
#define WEAK __attribute__ ((weak))
//-----------------------------------------------------------------------------
LPUART_Type* _celpuart[] = {LPUART0, LPUART1, LPUART2, LPUART3, LPUART4};
IRQn_Type _ce_lpuart_interrupt[] = {LPUART0_IRQn, LPUART1_IRQn, LPUART2_IRQn, LPUART3_IRQn, LPUART4_IRQn};
ceCom_RxCB_t _ce_rx_cb[] = {NULL, NULL, NULL, NULL, NULL};
cerb_t* rRB[] = {NULL, NULL, NULL, NULL, NULL};
cerb_t* tRB[] = {NULL, NULL, NULL, NULL, NULL};
//-----------------------------------------------------------------------------
// init
void ceComInit(uint32_t uartNo, lpuart_config_t config, size_t bufSize, ceCom_RxCB_t cbFuncOnRx)
{
	rRB[uartNo] = cerb_Create(bufSize);
	tRB[uartNo] = cerb_Create(bufSize);

	LPUART_Init(_celpuart[uartNo], &config, CECOM_CLK_FREQ);
	/* Enable RX interrupt. */
	LPUART_EnableInterrupts(_celpuart[uartNo], kLPUART_RxDataRegFullInterruptEnable);
	EnableIRQ(_ce_lpuart_interrupt[uartNo]);
	_ce_rx_cb[uartNo] = cbFuncOnRx;
}

void ceComInitialize()
{
	lpuart_config_t config;
	CLOCK_SetLpuartClock(2U);
#if(CECOM0_ENABLE==1)
//  Port B Clock Gate Control: Clock enabled
//  CLOCK_EnableClock(kCLOCK_PortB);
//  PORTB16 (pin ?) is configured as LPUART0_RX
//  PORT_SetPinMux(PORTC, 14U, kPORT_MuxAlt3);
//  PORTB17 (pin ?) is configured as LPUART0_TX
//  PORT_SetPinMux(PORTC, 15U, kPORT_MuxAlt3);
	LPUART_GetDefaultConfig(&config);
	config.baudRate_Bps = 4800;
	config.enableTx     = true;
	config.enableRx     = true;
	config.parityMode = kLPUART_ParityEven;
	// uart0, buffer size 1023, call back 'ceComOnRx0' on receiving
	ceComInit(0, config, 1024, ceComOnRx0);
#endif
#if(CECOM1_ENABLE==1)
//  Port ? Clock Gate Control: Clock enabled
//  CLOCK_EnableClock(kCLOCK_Port?);
//  PORTB16 (pin ?) is configured as LPUART0_RX
//  PORT_SetPinMux(PORTC, 14U, kPORT_MuxAlt3);
//  PORTB17 (pin ?) is configured as LPUART0_TX
//  PORT_SetPinMux(PORTC, 15U, kPORT_MuxAlt3);
	LPUART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200U;
	config.enableTx     = true;
	config.enableRx     = true;
	config.parityMode = kLPUART_ParityDisabled;
	// uart1, buffer size 1023, call back 'ceComOnRx1' on receiving
	ceComInit(1, config, 1024, ceComOnRx1);
#endif
#if(CECOM2_ENABLE==1)
	// Port D Clock Gate Control: Clock enabled
	CLOCK_EnableClock(kCLOCK_PortD);

	// pin config
	const port_pin_config_t UART2_RX = {/* Internal pull-up/down resistor is disabled */
											 kPORT_PullDisable,
											 /* Fast slew rate is configured */
											 kPORT_FastSlewRate,
											 /* Passive filter is disabled */
											 kPORT_PassiveFilterDisable,
											 /* Open drain is disabled */
											 kPORT_OpenDrainDisable,
											 /* Low drive strength is configured */
											 kPORT_LowDriveStrength,
											 /* Pin is configured as LPUART2_RX */
											 kPORT_MuxAlt3,
											 /* Pin Control Register fields [15:0] are not locked */
											 kPORT_UnlockRegister};
	/* PORTD2 (pin ?) is configured as LPUART2_RX */
	PORT_SetPinConfig(PORTD, 2U, &UART2_RX);

	const port_pin_config_t UART2_TX = {/* Internal pull-up/down resistor is disabled */
											 kPORT_PullDisable,
											 /* Fast slew rate is configured */
											 kPORT_FastSlewRate,
											 /* Passive filter is disabled */
											 kPORT_PassiveFilterDisable,
											 /* Open drain is disabled */
											 kPORT_OpenDrainDisable,
											 /* Low drive strength is configured */
											 kPORT_LowDriveStrength,
											 /* Pin is configured as LPUART2_TX */
											 kPORT_MuxAlt3,
											 /* Pin Control Register fields [15:0] are not locked */
											 kPORT_UnlockRegister};
	/* PORTD3 (pin ?) is configured as LPUART2_TX */
	PORT_SetPinConfig(PORTD, 3U, &UART2_TX);

	// pin mode
	/* PORTD2 (pin ?) is configured as LPUART2_RX */
	PORT_SetPinMux(PORTD, 2U, kPORT_MuxAlt3);

	/* PORTD3 (pin ?) is configured as LPUART2_TX */
	PORT_SetPinMux(PORTD, 3U, kPORT_MuxAlt3);

	LPUART_GetDefaultConfig(&config);
	config.baudRate_Bps = 8064U;
	config.parityMode = kLPUART_ParityEven;
	config.stopBitCount = kLPUART_TwoStopBit;
	config.enableTx     = true;
	config.enableRx     = true;
	// uart2, buffer size 1023, call back 'ceComOnRx2' on receiving
	ceComInit(2, config, 1024, ceComOnRx2);
#endif
#if(CECOM3_ENABLE==1)
//  Port ? Clock Gate Control: Clock enabled
//  CLOCK_EnableClock(kCLOCK_Port?);
//  PORTB16 (pin ?) is configured as LPUART0_RX
//  PORT_SetPinMux(PORTC, 14U, kPORT_MuxAlt3);
//  PORTB17 (pin ?) is configured as LPUART0_TX
//  PORT_SetPinMux(PORTC, 15U, kPORT_MuxAlt3);
	LPUART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200U;
	config.enableTx     = true;
	config.enableRx     = true;
	config.parityMode = kLPUART_ParityDisabled;
	// uart3, buffer size 1023, call back 'ceComOnRx3' on receiving
	ceComInit(3, config, 1024, ceComOnRx3);
#endif
#if(CECOM4_ENABLE==1)
	//  pin assignments for com4 in board init
	//  Port C Clock Gate Control: Clock enabled
	CLOCK_EnableClock(kCLOCK_PortC);
	//  PORTC14 (pin 86) is configured as LPUART4_RX
	PORT_SetPinMux(PORTC, 14U, kPORT_MuxAlt3);
	//  PORTC15 (pin 87) is configured as LPUART4_TX
	PORT_SetPinMux(PORTC, 15U, kPORT_MuxAlt3);

	LPUART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200U;
	config.enableTx     = true;
	config.enableRx     = true;
	config.parityMode = kLPUART_ParityDisabled;
	// uart4, buffer size 1023, call back 'ceComOnRx4' on receiving char
	ceComInit(4, config, 1024, ceComOnRx4);
#endif
}
//-----------------------------------------------------------------------------
#if(CECOM0_ENABLE==1)
void COM_IRQHandler0(void)
{
    uint8_t data;
    /* If new data arrived. */
    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(_celpuart[0]))
    {
        data = LPUART_ReadByte(_celpuart[0]);
        cerb_Push(rRB[0],data);
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif
#if(CECOM1_ENABLE==1)
void COM_IRQHandler1(void)
{
    uint8_t data;
    /* If new data arrived. */
    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(_celpuart[1]))
    {
        data = LPUART_ReadByte(_celpuart[1]);
        cerb_Push(rRB[1],data);
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif
#if(CECOM2_ENABLE==1)
void COM_IRQHandler2(void)
{
    uint8_t data;
    /* If new data arrived. */
    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(_celpuart[2]))
    {
        data = LPUART_ReadByte(_celpuart[2]);
        cerb_Push(rRB[2],data);
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif
#if(CECOM3_ENABLE==1)
void COM_IRQHandler3(void)
{
    uint8_t data;
    /* If new data arrived. */
    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(_celpuart[3]))
    {
        data = LPUART_ReadByte(_celpuart[3]);
        cerb_Push(rRB[3],data);
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif
#if(CECOM4_ENABLE==1)
void COM_IRQHandler4(void)
{
    uint8_t data;
    /* If new data arrived. */
    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(_celpuart[4]))
    {
        data = LPUART_ReadByte(_celpuart[4]);
        cerb_Push(rRB[4],data);
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif
//-----------------------------------------------------------------------------
void ceComTask()
{
#if(CECOM0_ENABLE==1)
	ceComTaskTx(0);
	ceComTaskRx(0);
#endif
#if(CECOM1_ENABLE==1)
	ceComTaskTx(1);
	ceComTaskRx(1);
#endif
#if(CECOM2_ENABLE==1)
	ceComTaskTx(2);
	ceComTaskRx(2);
#endif
#if(CECOM3_ENABLE==1)
	ceComTaskTx(3);
	ceComTaskRx(3);
#endif
#if(CECOM4_ENABLE==1)
	ceComTaskTx(4);
	ceComTaskRx(4);
#endif
}
//-----------------------------------------------------------------------------
void ceComTaskTx(uint32_t uartNo)
{
	// Send data only when LPUART TX register is empty and ring buffer has data to send out.
	while ((kLPUART_TxDataRegEmptyFlag & LPUART_GetStatusFlags(_celpuart[uartNo])) && (!cerb_IsEmpty(tRB[uartNo])))
	{
		LPUART_WriteByte(_celpuart[uartNo],cerb_Pop(tRB[uartNo]));
	}
}
//-----------------------------------------------------------------------------
//Polled regularly by main(), check the serial input buffer if
//there is any input char and then pass these chars to corresponding call back func
void ceComTaskRx(uint32_t uartNo)
{
	//while rx buf is not empty
    while(!cerb_IsEmpty(rRB[uartNo])) _ce_rx_cb[uartNo](cerb_Pop(rRB[uartNo]));
}
//-----------------------------------------------------------------------------
//put the char into the transmit buffer and start transmission
uint8_t ceComPutch(uint32_t uartNo, uint8_t c)
{
//  while(cerb_IsFull(tRB[uartNo]));//wait when the buffer is full
//	commenting above line means: Discard Policy=> discard old data
    cerb_Push(tRB[uartNo],c);
    return c;
}
//-----------------------------------------------------------------------------
void ceComPrint(uint32_t uartNo, char *s)
{
    for(;*s;s++) ceComPutch(uartNo,*s);
}
//-----------------------------------------------------------------------------

void ceComWrite(uint32_t uartNo, uint8_t *s,size_t n)
{
	size_t i;
	for(i=0;i<n;i++){
		ceComPutch(uartNo,s[i]);
	}
}
//-----------------------------------------------------------------------------
void ceComPrintHex(uint32_t uartNo, uint8_t ch)
{
    uint8_t a;
    a=(ch >> 4) | 0x30;
    if(a>0x39) a+=7;
    ceComPutch(uartNo,a);
    a=(ch & 0x0F) | 0x30;
    if(a>0x39) a+=7;
    ceComPutch(uartNo,a);
}

//-----------------------------------------------------------------------------
// Print 32 bit number in hex
void ceComPrintHex32(uint32_t uartNo, uint32_t r)
{
	int i;
	uint8_t b[4];
	for(i=0;i<4;i++){
		b[i] = (uint8_t)( (r>>(i*8)) & 0xFF);
	}
	for(i=0;i<4;i++){
		ceComPrintHex(uartNo,b[3-i]);
	}
}
//-----------------------------------------------------------------------------
void ceComWUT(uint32_t uartNo)
{
	while(!cerb_IsEmpty(tRB[uartNo])) ceComTaskTx(uartNo);
}
//-----------------------------------------------------------------------------
#if(CECOM0_ENABLE==1)
WEAK void ceComOnRx0(uint8_t ch)
{
// to do on rx
}
#endif
//-----------------------------------------------------------------------------
#if(CECOM1_ENABLE==1)
WEAK void ceComOnRx1(uint8_t ch)
{
// to do on rx
}
#endif
//-----------------------------------------------------------------------------
#if(CECOM2_ENABLE==1)
WEAK void ceComOnRx2(uint8_t ch)
{
// to do on rx
}
#endif
//-----------------------------------------------------------------------------
#if(CECOM3_ENABLE==1)
WEAK void ceComOnRx3(uint8_t ch)
{
// to do on rx
}
#endif
//-----------------------------------------------------------------------------
#if(CECOM4_ENABLE==1)
WEAK void ceComOnRx4(uint8_t ch)
{
// to do on rx
}
#endif
//-----------------------------------------------------------------------------
