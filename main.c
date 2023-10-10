/*
Code Credits:
 The following code developed based on below references
 1. Tiva LaunchPad: Internet of Things (IoT) with ESP8266 WiFi
		https://www.hackster.io/selcuk-cakmak/tiva-launchpad-internet-of-things-iot-with-esp8266-wifi-3e00e2
 2. Wifi -Module Reference 
	REAL-TIME INTERFACING TO ARM CORTEXTM-M MICROCONTROLLERS  (Volume-2 ) by Jonathan volvano Chapter-11.4
 3. ESP8266 resources below:
// General info and AT commands: http://nurdspace.nl/ESP8266
// General info and AT commands: http://www.electrodragon.com/w/Wi07c
// Community forum: http://www.esp8266.com/
// Offical forum: http://bbs.espressif.com/
// example http://zeflo.com/2014/esp8266-weather-display/	
 4. Servo Motor Reference -TM4C123G LaunchPad Workshop
 5. ADC Sampling, Interrupts  - ilearn examples

	Created on: Nov 14, 2022
  Author: Akshaya
 */
//--------- HEADER SECTION -----------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/fpu.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "DelayTimer.h"
#include "DelayTimer.c"
#include "light.c"
#include "dht22.c"

//--------- VARIABLE DECLARATION -----------------------------------------------
static char ReceivedData[512];
static char str[128];
static unsigned long timer_period = 16000000; //reload value to Timer0A to generate one second counter
char *ssid = "LAPTOP-ST-WIN-0 7808"; // ssid of the wifi network to be connected into.
char *password = "wZ28732?"; // password of the wifi network 

char* url = "api.openweathermap.org"; // url of openweathermap.org
char* url_server = "api.thingspeak.com";// url of thinkspeak for hosting data
char* port = "80";
char* fetch = "GET /data/2.5/weather?lat=37.774929&lon=-122.419418&units=metric&APPID=d5f17d0f36eb861814c9a3c0fa5bd9bb HTTP/1.1\r\nHost:api.openweathermap.org\r\n\r\n";
char* post_data = "GET /update?api_key=HK6L5I61RBTBIQBI HTTP/1.1\r\nHost:api.thingspeak.com\r\n\r\n";
bool process = true;
bool neg = false;
bool get_sensordata = true;// flag to initiate wifi and sensor data fetch
volatile int secs_counter = 0;// counter for 5 mins timer interrupt
volatile float temp_out, humidity_out;// temperature & humidity data (in integeer ) extracted from WIFI char data 

//--------- Function DECLARATION -----------------------------------------------
bool sendDataToServer(char* data, char* url, char *port);
char* buildDataToSend(char *base_data, char *temp_out, char *humid_out, char *temp_in, char *humid_in, char data[]);
bool getDataFromServer(void);
bool ExtractOutdoor(void);
char* GetWeatherData(void);
bool ProcessRoutine(void);
char *Substring(char *src, char *dst, int start, int stop);
int SearchIndexOf(char src[], char str[]);
char* itoa(int i, char b[]);
void ftoa(float f,char *buf);
bool recvAndFilter(char src[],char *begin, char* end, char data[], int timeout);
bool arrtofloat(char arr[], float *out);


// --------- BASE CODE BEGIN -----------------------------------------------
//--------- ESP8266 DRIVER -----------------------------------------------
void InitUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}

void InitESPUART(void)
{
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
	UARTConfigSetExpClk(UART1_BASE, 16000000, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTEnable(UART1_BASE);

}

void SendATCommand(char *cmd)
{
	while(UARTBusy(UART1_BASE));
	while(*cmd != '\0')
	{
		UARTCharPut(UART1_BASE, *cmd++);
		}
	UARTCharPut(UART1_BASE, '\r'); //CR
	UARTCharPut(UART1_BASE, '\n'); //LF

}

int recvString(char *target, char *data, int timeout, bool check)
{
	int i=0;
	char a;
	unsigned long start = millis();

    while (millis() - start < timeout)
    {
    	while(UARTCharsAvail(UART1_BASE))
    	{
              a = UARTCharGet(UART1_BASE);
              if(a == '\0') continue;
              data[i]= a;
              i++;
    	}
    	if(check)
    	{
    		if (SearchIndexOf(data, target) != -1)
    		{
    			break;
    		}
    	}
    }
    return 0;
}

bool recvFind(char *target, int timeout,bool check)
{
	recvString(target, ReceivedData, timeout, check);

	if (SearchIndexOf(ReceivedData, target) != -1)
	{
		return true;
	}
	return false;
}

bool recvFindAndFilter(char *target, char *begin, char* end, char *data, int timeout)
{
	recvString(target, ReceivedData, timeout, true);

    if (SearchIndexOf(ReceivedData, target) != -1)
    {
         int index1 = SearchIndexOf(ReceivedData, begin);
         int index2 = SearchIndexOf(ReceivedData, end);

         if (index1 != -1 && index2 != -1)
         {
             index1 += strlen(begin);
             Substring(ReceivedData,data,index1, index2);
             return true;
         }
     }
     data = "";
     return false;
}

bool ATesp(void)
{
	memset(ReceivedData, 0, sizeof(ReceivedData));
	SendATCommand("AT");
	return recvFind("OK",5000, true);
}

bool RSTesp(void)
{
	memset(ReceivedData, 0, sizeof(ReceivedData));
	SendATCommand("AT+RST");
	return recvFind("OK",5000, false);
}

bool CWQAPesp(void)
{
	memset(ReceivedData, 0, sizeof(ReceivedData));
	SendATCommand("AT+CWQAP");
	return recvFind("OK",10000, true);
}

bool CIPMUXesp(void)
{
	memset(ReceivedData, 0, sizeof(ReceivedData));
	SendATCommand("AT+CIPMUX=0");
	return recvFind("OK",5000, true);
}

bool ATGMResp(char *version)
{
	memset(ReceivedData, 0, sizeof(ReceivedData));
	SendATCommand("AT+GMR");
	return recvFindAndFilter("OK", "\r\r\n", "\r\n\r\nOK", version,10000);
}

bool aCWMODEesp(char *list)
{
	memset(ReceivedData, 0, sizeof(ReceivedData));
	SendATCommand("AT+CWMODE=?");
	return recvFindAndFilter("OK", "+CWMODE:(", ")\r\n\r\nOK", list,10000);
}

bool aCWLAPesp(char *list)
{
	memset(ReceivedData, 0, sizeof(ReceivedData));
	SendATCommand("AT+CWLAP");
	return recvFindAndFilter("OK","\r\r\n", "\r\n\r\nOK", list,15000);
}

bool aCIFSResp(char *list)
{
	memset(ReceivedData, 0, sizeof(ReceivedData));
	SendATCommand("AT+CIFSR");
	return recvFindAndFilter("OK","\r\r\n", "\r\n\r\nOK", list,15000);
}

bool CIPSTOesp(void)
{
	memset(ReceivedData, 0, sizeof(ReceivedData));
	SendATCommand("AT+CIPSTO=10000");
	return recvFind("OK",2000, true);
}



bool CIPCLOSEesp(void)
{
	memset(ReceivedData, 0, sizeof(ReceivedData));
	SendATCommand("AT+CIPCLOSE");
	return recvFind("OK",5000, true);
}

bool CIPSENDesp(char *text)
{
	int len = strlen(text)+2;

	itoa(len,str);

	char* AT_CMD_SEND = "AT+CIPSEND=";
	char CMD_TEXT[128];
	strcpy(CMD_TEXT,AT_CMD_SEND);
	strcat(CMD_TEXT,str);
	//UARTprintf(CMD_TEXT); //debug
	memset(ReceivedData, 0, sizeof(ReceivedData));
	SendATCommand(CMD_TEXT);
	delay(50);
	//UARTprintf(text); //debug
	SendATCommand(text);
	return recvFind("SEND OK",10000, true);
}

void HardwareReset()
{
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5); //Output
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00); //LOW ->Reset to ESP8266
	delay(50);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5); //Output ->// Open drain; reset -> GND
	delay(10);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_5); //Input ->// Back to high-impedance pin state
	delay(3000);
}

void QuitProcess(void)
{
	process = false;
}
//--------- STRING OPERATION -----------------------------------------------
char *Substring(char *src, char *dst, int start, int stop)
{
	int len = stop - start;
	strncpy(dst, src + start, len);

	return dst;
}

int SearchIndexOf(char src[], char str[])
{
   int i, j, firstOcc;
   i = 0, j = 0;
   while (src[i] != '\0')
   {

      while (src[i] != str[0] && src[i] != '\0')
         i++;

      if (src[i] == '\0')
         return (-1);

      firstOcc = i;

      while (src[i] == str[j] && src[i] != '\0' && str[j] != '\0')
      {
         i++;
         j++;
				
      }

      if (str[j] == '\0')
         return (firstOcc);
      if (src[i] == '\0')
         return (-1);

      i = firstOcc + 1;
      j = 0;
   }

   return (-1);
}

char* itoa(int i, char b[])
{
    char const digit[] = "0123456789";
    char* p = b;
    if(i<0){
        *p++ = '-';
        i *= -1;
    }
    int shifter = i;
    do{
        ++p;
        shifter = shifter/10;
    }while(shifter);
    *p = '\0';
    do{
        *--p = digit[i%10];
        i = i/10;
    }while(i);
    return b;
}

void ftoa(float f,char *buf)
{
    int pos=0,ix,dp,num;
    if (f<0)
    {
        buf[pos++]='-';
        f = -f;
    }
    dp=0;
    while (f>=10)
    {
        f=f/10;
        dp++;
    }
    for (ix=1;ix<8;ix++)
    {
            num = f;
            f=f-num;
            if (num>9)
                buf[pos++]='#';
            else
                buf[pos++]='0'+num;
            if (dp==0) buf[pos++]='.';
            f=f*10;
            dp--;
    }
}

//--------- SERVO DRIVER -----------------------------------------------
/*void InitServo(void) //PC5
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinConfigure(GPIO_PC5_WT0CCP1);
	GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_5);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
	TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_PWM);
	TimerLoadSet(WTIMER0_BASE, TIMER_B, (Period-1));
	TimerMatchSet(WTIMER0_BASE, TIMER_B, (Period-9600));

	TimerMatchSet(WTIMER0_BASE, TIMER_B, 1);
	TimerEnable(WTIMER0_BASE, TIMER_B);
}

void ConfigureServo(void)
{
    unsigned int servo_stepval, servo_stepnow;
    unsigned int i;

    servo_stepval   = ( (SERVO_MAX - SERVO_MIN) / SERVO_STEPS );
    servo_stepnow   = SERVO_MIN;

    for (i = 0; i < (SERVO_STEPS+1); i++)
    {
    	servo_lut[i] = (Period-servo_stepnow);
    	servo_stepnow += servo_stepval;
    }
}

void SetServoPosition(uint32_t position)
{
	TimerMatchSet(WTIMER0_BASE, TIMER_B, position);
}

void SetServoAngle(uint32_t angle)
{
	SetServoPosition(servo_lut[angle]);
}*/
//--------- BASE CODE END  ----------------------------------------------------
//--------- MY CODE ----------------------------------------------------
// function to set Wifi module mode
bool CWMODEesp(char* mode)
{
	memset(ReceivedData, 0, sizeof(ReceivedData));
	char *command = "AT+CWMODE=";
	char COMMAND[128];
	strcpy(COMMAND, command);
	strcat(COMMAND, mode);
	SendATCommand(COMMAND);
	UARTprintf("\n\rSetting mode to client \n\r");
	return recvFind("OK",5000, true);
}
// function to establish wifi connection
bool CWJAPesp(char* ssid, char* password)
{
	memset(ReceivedData, 0, sizeof(ReceivedData));
	char command[128];
	strcpy(command, "AT+CWJAP=\"");
	strcat(command, ssid);
	strcat(command, "\",\"");
	strcat(command, password);
	strcat(command, "\"");
	//UARTprintf(command); //debug
	SendATCommand(command);
	//SendATCommand("AT+CWJAP=\"****\",\"****\""); //Your Wifi: NetworkName, Password
	return recvFind("OK",10000, true);
}
// Function to  start TCP server 
bool CIPSTARTesp(char* url, char* port)
{
	memset(ReceivedData, 0, sizeof(ReceivedData));
	char command[128]; 
	strcpy(command,"AT+CIPSTART=\"TCP\",\"");
	strcat(command, url);
	strcat(command,"\",");
	strcat(command, port);
	//UARTprintf(command); //debug
	SendATCommand(command);
	//SendATCommand("AT+CIPSTART=\"TCP\",\"api.openweathermap.org\",80\r\n"); //Server IP and Port: such as 192.255.0.100, 9999
	return recvFind("OK",5000, true);
}
//Globally enable interrupts 
void IntGlobalEnable(void)
{
    __asm("    cpsie   i\n");
}
// Timer0A interrupt for 5 minutes - to get wifi & sensor data every 5 mintes
void Timer0A_Init(unsigned long period)
{   
	
  // Enable Peripheral Clocks 
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); 		// configure for 32-bit timer mode
  TimerLoadSet(TIMER0_BASE, TIMER_A, period -1);      //reload value
	IntPrioritySet(INT_TIMER0A, 0x00);  	 // configure Timer0A interrupt priority as 0
  IntEnable(INT_TIMER0A);    				// enable interrupt 19 in NVIC (Timer0A)
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);      // arm timeout interrupt
  TimerEnable(TIMER0_BASE, TIMER_A);      // enable timer0A
}

//interrupt handler for Timer0A
void Timer0A_Handler(void)
{
		// acknowledge flag for Timer0A timeout
		TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
		secs_counter++;
		if(secs_counter == 300){ // Counter for 5 minutes 
		get_sensordata = true; // flage set for data fetch
		secs_counter = 0;
		}

}


int main(void)
{
	bool task_status;
  // System clock at 16MHz
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  // Port Initialization
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0x00);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x02);

	TimerInit();// Timer5A initialization for delay functions
	HardwareReset();// Reset  for WIFI module
	InitUART();// UART initialization
	InitESPUART();// ESP8266 UART initialization
	Light(); // ADC initialization for photo resistor
	Timer0A_Init(timer_period);
	UARTprintf("\n\r\n\rENGR 844 FINAL EXAM - Akshaya Sekar & Niranjana!\n");
	delay(10);
	
	while(1){
	if(get_sensordata){
	task_status = ATesp();
	if(task_status)
		UARTprintf("\n\rAT Command is working ! \n\r"); //debug
	else
		UARTprintf("\n\rAT Command is not working ! \n\r"); //debug
	task_status = RSTesp();
	if(task_status)
		UARTprintf("\n\rRT Command is working ! \n\r"); //debug
	else
		UARTprintf("\n\rRT Command is not working ! \n\r"); //debug
	task_status = CWMODEesp("1");
	if(task_status)
		UARTprintf("\n\rAT Mode set to Client \n\r"); //debug
	else
		UARTprintf("\n\rAT Mode not set to Client ! \n\r"); //debug
	
	while(true)
	{
		UARTprintf("\nTrying to connect wi-fi\n");
		task_status = CWJAPesp(ssid, password);
		if(task_status)
		{
			UARTprintf("\nConnection is established!\n");
			break;
		}
	}
	// Led toggle for Status Notification 
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00); // Turn off Red LED
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x04); //Turn on Blue LED
	UARTprintf("\nTest Connect to server\n"); //debug
	task_status = CIPSTARTesp(url, port); 
	if(task_status)
		UARTprintf("\nTest Connection to server sucessful!\n"); //debug
	else
		UARTprintf("\nTest Connection to server failed!\n"); //debug
	task_status = ProcessRoutine();// fetch data from openweathermap
	UARTprintf("\n"); //debug
	// Extraction of Temperature & Humidity data from server response 
	int i=0;
	char temp_data[512];
	while(ReceivedData[i]!='\0')
	{
		temp_data[i] = ReceivedData[i];
		i++;
	}
	
	char temperature[5]; 
	char humidity[2];
	bool status = recvAndFilter(temp_data,"temp\":", ",\"feels_like\":", temperature, 10000);
	if (status){
		UARTprintf("\n\rOutside Temperature: %s",temperature); //debug
		UARTprintf("\n\r"); //debug
	}
	status = false;
	status = recvAndFilter(temp_data,"\"humidity\":", "},\"visibility\"", humidity, 10000);
	if (status){
		UARTprintf("\n\rOutside Humidity: %s",humidity); //debug
		//UARTprintf(humidity); //debug
		UARTprintf("\n\r"); //debug
	}
	/* Convert char to float */
	status = false;
	temp_out = 0.0;
	humidity_out = 0.0;
	
	float *temp_tmp, *humid;
	status = arrtofloat(temperature,temp_tmp);
	status = arrtofloat(humidity,humid);
	temp_out = *temp_tmp;
	humidity_out = *humid;
	//ExtractOutdoor();
	UARTprintf("\nOutdoor Condition Extracted Successfully!!\n"); //debug
	if(task_status)
		{
			CIPCLOSEesp();
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00); //Turn off Green Led
			UARTprintf("\nTCP Connection closed\n"); //debug
		}
		else
		{
			delay(10);
			UARTprintf("\nBug - Unknown bug in Code\n"); //debug
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00); //Turn off Blue Led
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x02); //Turn on Red Led
		}
		
	UARTprintf("\n\rNursery Conditions\n\r");
		
	float *temp_in, *humid_in;
	int nursery_temp = nurseryTemp();
	int nursery_humid = nurseryHumidity();
	//UARTprintf("Nursery Params : %d , %d",nursery_temp,nursery_humid); //debug
	if(nursery_temp > temp_out){
		UARTprintf("\nOpen Door !! Turn Heater off!! \n");
		servo(true);
	}
	else{
		UARTprintf("\nClose Door !! Turn Heater on!! \n");
		servo(false);
	}
	if(humidity_out > nursery_humid){
		UARTprintf("\nTurn off De-humidier!! \t Turn on Humidifier!! \n");
	}else{
		UARTprintf("\nTurn on De-humidier!! \t Turn off Humidifier!! \n");
	}
	get_sensordata = false;
	// Send Nursery & werather data to Thinkspeak server for Remote Monitoring
	//char *humidity_in, *temperature_in, out_data[512],*out_data_temp;
	//buildDataToSend(post_data,temperature,humidity,humidity_in,temperature_in,out_data);
	//sendDataToServer(out_data,url_server,port);
	}
	controlLight();// LED brightness control using PWM based on photoresistor value
}
}

bool ProcessRoutine()
{
	/*
	A function to run the fetch command & wait for the data from server
	Once the data is received use a "specific word" search to exit from wait
	*/
	bool status;
	process = true;
	UARTprintf("Waiting Server...\n");
	while(true)
	{
		status = CIPSTARTesp(url,port);
		if(status)
		{
			UARTprintf("Communication is established!\n");
			break;
		}

		delay(20);
	}
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x08); //TurnOn Green Led-> READY

	int i=0;
	char a;
	memset(ReceivedData, 0, sizeof(ReceivedData)); //Clear
	unsigned long start;
	bool task_status = CIPSENDesp(fetch);
	if (task_status){
		UARTprintf("\n\rRequest sent to openweather.org\n");
	}
			
	while(process)
	{
		if(UARTCharsAvail(UART1_BASE))
		{
			if (SearchIndexOf(ReceivedData, "+IPD,") != -1)
			{
				i=0;
				memset(ReceivedData, 0, sizeof(ReceivedData));

				start = millis();

			    while (millis() - start < 8000)
			    {
			    	while(UARTCharsAvail(UART1_BASE))
			    	{
			    		a = UARTCharGet(UART1_BASE);
			    		if(a == '\0') continue;
			    		ReceivedData[i]= a;
			    		i++;
			    	}
		    		if (SearchIndexOf(ReceivedData, "cod") != -1)
		    		{
		    			break;
		    		}
			    }
				UARTprintf("\n\r");
				UARTprintf(ReceivedData);
				UARTprintf("\n\r");
				return true;
			}
			else
			{
				a = UARTCharGet(UART1_BASE);
				if(a == '\0') continue;
				ReceivedData[i]= a;
				i++;
			}
		}
	}
	return true;
}


bool getDataFromServer(void){
  int i;
  char a;
	memset(ReceivedData, 0, sizeof(ReceivedData)); //Clear
	unsigned long start;
	
  while(process)
	{
		if(UARTCharsAvail(UART1_BASE))
		{
			if (SearchIndexOf(ReceivedData, "+IPD,") != -1)
			{
				i=0;
				memset(ReceivedData, 0, sizeof(ReceivedData));

				start = millis();

			    while (millis() - start < 8000) // wait for 8 secs
			    {
			    	while(UARTCharsAvail(UART1_BASE))
			    	{
			    		a = UARTCharGet(UART1_BASE);
			    		if(a == '\0') continue;
			    		ReceivedData[i]= a;
			    		i++;
			    	}

		    		if (SearchIndexOf(ReceivedData, "cod") != -1)
		    		{
		    			break;
		    		}
			    }
		    UARTprintf("\n\r"); //debug
				UARTprintf(ReceivedData); //debug
				UARTprintf("\n\r"); //debug
				return true;
			}
			else
			{
				a = UARTCharGet(UART1_BASE);
				if(a == '\0') continue;
				ReceivedData[i]= a;
				i++;
			}
		}
	}
	
	return true;
	
}


bool ExtractOutdoor(void){
	/*
	Extract Ambient Temperature and Humidity from Server data and convert it into 
	float
	*/
	int i=0;
	char temp_data[512];
	while(ReceivedData[i]!='\0')
	{
		temp_data[i] = ReceivedData[i];
		i++;
	}
	//UARTprintf(temp_data);
	char temperature[5]; 
	char humidity[2];
	bool status = recvAndFilter(temp_data,"temp\":", ",\"feels_like\":", temperature, 10000);
	if (status){
		UARTprintf("\n\rOutside Temperature: %s",temperature); //debug
		//UARTprintf(temperature); //debug
		UARTprintf("\n\r"); //debug
	}
	status = false;
	status = recvAndFilter(temp_data,"\"humidity\":", "},\"visibility\"", humidity, 10000);
	if (status){
		UARTprintf("\n\rOutside Humidity: %s",humidity); //debug
		//UARTprintf(humidity); //debug
		UARTprintf("\n\r"); //debug
	}
	/* Convert char to float */
	status = false;
	temp_out=0.0;
	humidity_out=0.0;
	
	float *temp_tmp, *humid;
	status = arrtofloat(temperature,temp_tmp);
	status = arrtofloat(humidity,humid);
	temp_out = *temp_tmp;
	humidity_out = *humid;
	UARTprintf("\n Exit from Extraction \n");
	return 0;
}

char* buildDataToSend(char *base_data, char *temp_out, char *humid_out, char *temp_in, char *humid_in, char data[]){
    /*
    Build from base data with field values that could be used by ThinkSpeak to
    display data for user interaction
    Data Sent: Outside Temperature, Outside Humidity, Nursery Temperature, Nursery Humidity, Ambient Light  
    */
    //char out[1000];
    strcpy(data,base_data);
    strcat(data,"&field1=");
    strcat(data, temp_out);
    strcat(data,"&field2=");
    strcat(data, humid_out);
    strcat(data,"&field3=");
    strcat(data, temp_in);
    strcat(data,"&field4=");
    strcat(data, humid_in);
    UARTprintf("\n\r"); //debug
    UARTprintf(data); //debug
    UARTprintf("\n\r"); //debug
    return data;
}

bool sendDataToServer(char* data, char* url, char *port){
    /*
    Send data to the server when data, server url, and server port number are provided
    */
    bool status;
    // Set Wifi to client mode
    status = CWMODEesp("1");
    if (status)
    UARTprintf("\nMode Set to Client \n");
    // Connect to server
    UARTprintf("Waiting for ThinkSpeak...\n");

	while(true)
	{
		status = CIPSTARTesp(url,port);
		if(status)
		{
			UARTprintf("Communication established with ThinkSpeak!\n");
			break;
		}

		delay(50);
	}
    // Send data to server
    status = CIPSENDesp(post_data);
	if (status){
		UARTprintf("\n\r Data sent to ThinkSpeak! \n");
	}
    // Close connection 
    status = CIPCLOSEesp();
	if (status){
		UARTprintf("\n\r Connection closed to ThinkSpeak! \n");
	}
	return true;
}

bool recvAndFilter(char src[],char *begin, char* end, char data[], int timeout)
{
		//  Function to filter desired data  based on previous & next word
		 int index1 = SearchIndexOf(src, begin);
		 int index2 = SearchIndexOf(src, end);

		 if (index1 != -1 && index2 != -1)
		 {
				 index1 += strlen(begin);
				 Substring(src,data,index1, index2);
				 return true;
		 }
     
     data = "";
     return false;
}
bool arrtofloat(char arr[], float *out){
	/*
	A function to convert arry of string into float
	*/
	int temp,dec = 0;
	char *data_ptr;
	bool temp_neg = false;
	int test_bug = 0; //debug
	for(data_ptr = arr; *data_ptr != '\0'; data_ptr++){
		if(*data_ptr == '.'){
			dec = 10;
		}
		else if(*data_ptr == '-'){
			temp_neg = true;continue;
		}else{
		temp = (int)*data_ptr;
		*out = *out*10 + temp-48;
		test_bug = test_bug*10 + temp-48;
		//UARTprintf("\nTest conversion %s : %d \n",arr,test_bug);
		if(dec > 1)
			dec *= 10;
		}
	}
	if(dec>0){
		*out /= (dec/10); // included fix for extra 10x by division 
		test_bug /= (dec/10); // included fix for extra 10x by division 
	}
	if(temp_neg){
		*out *= -1;
		test_bug *= -1;
	}
	//UARTprintf("\nTest conversion : %d \n",dec); // debug decimal divisor
	UARTprintf("\nTest conversion %s : %d \n",arr,test_bug); // debug conversion
	return true;
}
