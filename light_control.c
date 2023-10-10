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



void photresistor_Init()
{
SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3);
ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |ADC_CTL_END);
ADCSequenceEnable(ADC0_BASE, 3);
ADCIntClear(ADC0_BASE, 3);

}
	
uint32_t adcOutput()
{  
	uint32_t readValue;	
	ADCProcessorTrigger(ADC0_BASE, 3);
  while(!ADCIntStatus(ADC0_BASE, 3, false))
  {
  }
  ADCIntClear(ADC0_BASE, 3);
	readValue=ADC0_SSFIFO3_R&0xFFF;
  return readValue;
}
 void setDelay(uint32_t time)
{
	SysCtlDelay(time * (SysCtlClockGet() / 3 / 1000000));
}
 
void LED_setup( uint32_t period,uint32_t Adjust,uint32_t duty_value_1)
{
	//Configure PWM Clock divide system clock by 64
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PD0_M1PWM0); // SERVO
	GPIOPinConfigure(GPIO_PD1_M1PWM1); // LED	
	volatile uint32_t duty_value = Adjust * period / 1000 ; 
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, period);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, duty_value);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, duty_value_1);	
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);
		
}
