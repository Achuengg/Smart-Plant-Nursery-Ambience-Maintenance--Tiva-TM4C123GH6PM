/*
Below code utilized from
Tiva LaunchPad: Internet of Things (IoT) with ESP8266 WiFi
https://www.hackster.io/selcuk-cakmak/tiva-launchpad-internet-of-things-iot-with-esp8266-wifi-3e00e2
*/
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
//#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
//#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/hw_nvic.h"
#include "inc/tm4c123gh6pm.h"


static unsigned long milliseconds = 0;

void TimerInit(void)
{

	SysTickPeriodSet(0x00FFFFFF);
    SysTickEnable();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC_UP);

    TimerLoadSet(TIMER5_BASE, TIMER_A, SysCtlClockGet()/1000);

    IntEnable(INT_TIMER5A);
    TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

    TimerEnable(TIMER5_BASE, TIMER_A);

    IntMasterEnable();

}

unsigned long micros(void)
{
	return (milliseconds * 1000) + (HWREG(TIMER5_BASE + TIMER_O_TAV) / 80);
}

unsigned long millis(void)
{
	return milliseconds;
}

void delayMicroseconds(unsigned int us)
{
	volatile unsigned long elapsedTime;
	unsigned long startTime = HWREG(NVIC_ST_CURRENT);
	do{
		elapsedTime = startTime-(HWREG(NVIC_ST_CURRENT) & 0x00FFFFFF);
	}
	while(elapsedTime <= us*80);
}

void delay(uint32_t milliseconds)
{
		unsigned long i;
		for(i=0; i<milliseconds; i++){
			delayMicroseconds(1000);
		}
}

void Timer5IntHandler(void)
{
    TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

	milliseconds++;
}


