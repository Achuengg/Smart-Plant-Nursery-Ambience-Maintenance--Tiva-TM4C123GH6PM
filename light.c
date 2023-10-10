// Below code developed based on reference from ADC & PWM Examples in ilearn
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h" 
#include "driverlib/pin_map.h"
#include "inc/tm4c123gh6pm.h"
#include "light_control.c"

#ifdef DEBUG
void__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


#define PWM_FREQUENCY 50;
volatile float voltage;
volatile uint32_t period ,duty_value,duty_value_1 = 1;
volatile uint8_t adjust = 110;
uint16_t adcReadValue;
float resVar; float threshold= 0.05;// you can choose threshold based on your photo resistor and light
		

void Light(void)
{
	// Initialization
	uint32_t ui32PWMClock = SysCtlClockGet() / 64;
	period = (ui32PWMClock / 50) - 1;
	photresistor_Init();// ADC 
	LED_setup(period,adjust,duty_value_1);// PWM 
}

int controlLight(){
	// Function to read Photo resistor value (ADC) & control LED broghtness accordingly using PWM signal
	int resistor = 10000;
		adcReadValue = adcOutput();
		resVar = (4096*resistor/(adcReadValue+1))-resistor;//in case adcReadValue=0, this step i.e adding 1 in denominator makes sure that resVar is still defined
		voltage=3300/(resistor+resVar);
	  
		if (voltage < threshold) 
		{
			duty_value_1 = period *((threshold - voltage)/threshold);
			if(duty_value_1<1)
				duty_value_1 = 1;
			
		}			
		 else 
		 {
			 duty_value_1 = 1;
			 
			}	
		
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, duty_value_1);	//LED
		setDelay(10);//delay to get stable output or you can decrease this delay to get a faster response		
		
 return 0;
}

void servo(bool door_control)
{
	// PWM  for generation Servo Motor 
	if (door_control){
		if (adjust < 1)
		{
			adjust = 1;
		}
		adjust++;
		if (adjust > 64)
		{
			adjust = 64;
		}
	}
	else 
	 {
		 adjust--;
			}	
		
		duty_value = adjust * period / 1000 ; 
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0,duty_value );  //SERVO
}

	