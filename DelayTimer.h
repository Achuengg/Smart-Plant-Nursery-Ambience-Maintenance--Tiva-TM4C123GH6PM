/*
Below code utilized from
Tiva LaunchPad: Internet of Things (IoT) with ESP8266 WiFi
https://www.hackster.io/selcuk-cakmak/tiva-launchpad-internet-of-things-iot-with-esp8266-wifi-3e00e2
*/
#ifndef DELAYTIMER_H_
#define DELAYTIMER_H_

extern void TimerInit(void);
extern unsigned long micros(void);
extern unsigned long millis(void);
extern void delayMicroseconds(unsigned int us);
extern void delay(uint32_t milliseconds);

#endif /* DELAYTIMER_H_ */
