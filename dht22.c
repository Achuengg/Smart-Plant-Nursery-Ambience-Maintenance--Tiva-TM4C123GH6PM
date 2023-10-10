/*
Below code developed based on below reference
https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/
arm-based-microcontrollers/f/arm-based-microcontrollers-forum/956490/
ccs-tm4c123gh6pm-dht22-temp-and-humidity-sensor-read-values-0-0-c-and-0-0
*/

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.c"

// Constants for Temperature and Humidity Sensor
#define Clock_GPIO_C 0x04
#define DIR_PIN4_In ~0x10
#define DIR_PIN4_Out 0x10
#define High_In 0x10
#define Low_In ~0x10
#define High_Out 0x10
#define Low_Out 0x00
#define DEN_ENABLE_C4 0x10

// Variable Declaration
int temp1,temp2;

int Binary2Decimal(int Bit_L, int Bit_H, unsigned int Binary[])
{
// Function for Binary to decimal convertion	
int temp=0,i;
for(i = Bit_L;i<Bit_H;i++)
temp += Binary[i] * (1<<((Bit_H-1)-i)); //1 << n is the same as raising 2 to the power n, or 2^n
return temp;
}

/* This function generates the delay. */

void Delay(unsigned long int period)
{
int i=0;
for(i=0;i<12*(period/10);i++);
}


int nurseryTemp()
{
//---------------- Variables for Humidity and Temperature Sensor----------------------------------//

int i=0;
unsigned int Data[40];
int Temp_16, Temp_High, Temp_Low; // Variables for Temperature
int RH_16, RH_High, RH_Low; // Variables for Relative Humidity
int Check_Sum, sum; // Variables for Parity Checking

SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
Delay(100); // Delay to make Clock Smooth

//------------------Collecting the data from the sensor------------------------------------------//


while(1) // Iterates until temperature and humidity values are true
{
GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4); // Set Pin C4 as output
GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_4,Low_In);
Delay(1000);
GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_4,High_In);
Delay(30);
GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4); // Set Pin C4 as Input
while ((GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) == Low_Out) // Iterates untill data line is low
{}
while(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4) == High_Out) // Iterates untill data line is high
{}
while(i<40)
{ // Iterates to get 40 bits data from the sensor
while(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4) == Low_Out) // Iterates untill the data line is low
{}
Delay(45);
if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4) == Low_Out) // Check after 45us
Data[i] = 0; // if data line is low // 0 bit by sensor
else // else
Data[i] = 1; // 1 bit by sensor
while(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4) == High_Out) // Iterates till the data line is high
{}
i++;
}
//------------------Processing the data from the sensor------------------------------------------//
i=0;
RH_High = Binary2Decimal(0,8,Data); // Converts first 8 bits (Humidity bits) of the data into Decimal value
RH_Low = Binary2Decimal(8,16,Data); // Converts 8-16 bits of (Humidity bits) the data into Decimal value
Temp_High = Binary2Decimal(17,24,Data); // Converts 17-24 bits of (Temperature bits) the data into Decimal value
Temp_Low = Binary2Decimal(24,32,Data); // Converts 24-32 bits of (Temperature bits) the data into Decimal value
Check_Sum = Binary2Decimal(32,40,Data); // Converts last 8 bits of (Parity bits) the data into Decimal value
sum = RH_High + RH_Low + Temp_High + Temp_Low; // Sum the 8 high and 8 low bits of both humidity and temperature
if(sum>255)
sum -= 256; // To keep the sum 8 bit long
if(Check_Sum == sum) // Parity checking
break; // if no error, breaks the above while loop
}
RH_16 = Binary2Decimal(0,16,Data); // Converts Humidity bits into Decimal value
temp1 = RH_16/10;
Temp_16 = Binary2Decimal(17,32,Data); // Converts Temperature bits into Decimal value
temp2 = Temp_16/10;


//------------------Displaying the data from the sensor through UART and PuTTY------------------------------------------//

UARTprintf("Nursery Humidity=%d.%d Nursery Temperature=%d.%d \r\n", temp1,RH_16%10, temp2,Temp_16%10);
//*temp = temp2;
// UARTprintf(" H=%d T=%d\r\n", temp1/10+48,temp2/10+48 );
// Wait for two seconds
//Delay(2000000);
return temp2;// return temperature value 
}

int nurseryHumidity(){
	return temp1;// return humidity value 
}
