// Project: K82_Tmr_Com
// Author: Yan Naing Aye
// Last modified: 2021-06-30
// Description: K82 software timer and com example


#include "board.h"
#include "pin_mux.h"
#include "ceTmr.h"
#define BOARD_LED_GPIO     BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_PIN

void sby_led_to(void){
	GPIO_PortToggle(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
}

int main(void)
{
    BOARD_InitPins();
    BOARD_InitBootClocks();

    // Set systick reload value to generate 1ms interrupt
    if (SysTick_Config(SystemCoreClock / 1000U)) { while (1) { } }

    // create timer with interval 500 ms, not oneshot, call back function 'sby_led_to'
    ceTmr_t* sbyLED = ceTmrCreate(500,false,sby_led_to);
    ceTmrStart(sbyLED); // start the standby led timer

    while (1)
    {
        ceTmrTask();
    }
}
