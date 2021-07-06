/*
 * main.c
 *
 *  Description: ring buffered vcom
 *      Author: yanna
 */


#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"
#include "pin_mux.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "virtual_com.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);
void BOARD_DbgConsole_Deinit(void);
void BOARD_DbgConsole_Init(void);

#if defined(__CC_ARM) || (defined(__ARMCC_VERSION)) || defined(__GNUC__)
int main(void)
#else
void main(void)
#endif
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    APPInit();

    while (1)
    {
        APPTask();
    }
}
