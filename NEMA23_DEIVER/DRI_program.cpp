/****************************************************************************************************************************************
*****************************************************************************************************************************************
**************************************************    Date     :  11/15/2025			*************************************************
**************************************************    Name     :   Mario Emad Boles 	*************************************************
**************************************************    Version  :   1.0	            	*************************************************
**************************************************    SWC      :  NEMA23_DRIVER	        *************************************************
*****************************************************************************************************************************************
*****************************************************************************************************************************************
*/
//LIBS WE NEED FOR ESP32 FIRMWARE
#include <Arduino.h>
#include "DRI_interface.h"


void DRI_init(void)
{
	//SET THE PINS TO OUTPUT	
	pinMode(DIR_PIN,OUTPUT);
	pinMode(PUL_PIN,OUTPUT);
}

void DRI_riseup(void)//rise up
{

	//FIRST LETS SET THE DIRECTION TO CLOCKWISE
	digitalWrite(DIR_PIN,HIGH);//CLK WISE
	delayMicroseconds(5);//must to wait a 5 micro seconds for the driver to understand

	int check=0;

	while(DRI_STEPS>check)//this loop will run in the amount of the steps number
	{
		//HIGH THE POWERSCREW
		digitalWrite(PUL_PIN,HIGH);
		delayMicroseconds(10);
		digitalWrite(PUL_PIN,LOW);
		delayMicroseconds(10);
		check++;


		delayMicroseconds(2000);//change this number from 2000 --> 4000
	}

} 

void DRI_falldown(void)// fall down
{
	

	//FIRST LETS SET THE DIRECTION TO CONTERCLOCKWISE
	digitalWrite(DIR_PIN,LOW);//CONTERCLK WISE
	delayMicroseconds(5);//must to wait a 5 micro seconds for the driver to understand

	int check=0;

	while(DRI_STEPS>check)//this loop will run in the amount of the steps number
	{
		//LOW THE POWERSCREW
		digitalWrite(PUL_PIN,HIGH);
		delayMicroseconds(10);
		digitalWrite(PUL_PIN,LOW);
		delayMicroseconds(10);
		check++;


		delayMicroseconds(2000);//change this number from 2000 --> 4000
	}

}

