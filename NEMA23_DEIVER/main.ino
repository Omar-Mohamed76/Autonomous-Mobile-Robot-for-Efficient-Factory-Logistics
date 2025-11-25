
/****************************************************************************************************************************************
*****************************************************************************************************************************************
**************************************************    Date     :  11/15/2025			*************************************************
**************************************************    Name     :   Mario Emad Boles 	*************************************************
**************************************************    Version  :   1.0	            	*************************************************
**************************************************    SWC      :  NEMA23_DRIVER	        *************************************************
*****************************************************************************************************************************************
*****************************************************************************************************************************************
*/
#include "DRI_interface.hpp"
#include  <Arduino.h>


#define NUMBER_OF_REV   5
#define SW             18 // Switch 

void setup()
{
    Serial.begin(115200);
    Serial.println("NEMA23 Driver Test Starting...");
	
	//the limit switch pins
    pinMode(SW,INPUT);
	
    DRI_init();

}
void loop()
{
    //HERE DON'T FORGET TO ADD A COMMUNICATIONAL PRTOCLA METHOD SIGNAL
    //I WILL ASSUME THAT THE SIGNAL IS 1 FOR RISE UP
    //AND 0 FOR FALL DOWN
	
	

    int checker;
	int steps=0;

    //pass this v to the com protocols

    switch (checker)
    {
	//THE FIRST CASE IS FOR RISING 
	//WE WILL RISE HERE BY A FIXED NUMBER OF STEPS NO SWITCH
    case 1:
		Serial.println("Start the rising...");
		while(steps < NUMBER_OF_REV)//WHILE THE STEPS OPERATOR IS NOT EQUAL TO THE FIXED AMOUNT OF STEPS 
		{
			DRI_riseup();//ONE STEP
			steps++;
			Serial.println("This is the Rising Step number: %d\n",steps);//DEBUGING
		}
		steps=0;//RESTE THE STEPS ITER
        break;
	//IN THIS CASE WE WILL GO DOWN AND BY USING TO METHODS 
	//THE FIRST IS BY FIXED AMOUNT OF STEPS ------BUT THIS WILL MAKE A PROBLEM IS THERE IS ANY SKIPED STEPS--------
	//THE SEC IS BY PRESSING A SWITCH WHEN THE SCREW GO DOWN 
	//-----BUT THIS ALSO WILL HAVE A PROBLEM IF THE SWITCH DOSENT GIVE A SIGNAL THE MOTOR WILL BE FRIED------------
	//BY COMPINIG THIS METHODS TOGATHER THE PROBLEM IS SOLVED 
    case 2:
		Serial.println("Start the falling...");
		while(steps < NUMBER_OF_REV && digitalRead(SW)!= 1)//WHILE THE STEPS IS LESS THAN THE FIXED AND SW NOT PRESSED
			{
				DRI_falldown();//ONE STEP DOWN
				steps++;
				Serial.println("This is the Falling Step number: %d\n",steps);//DEBIGING
			}
		//THIS PART IS FOR DEBUGING IF THERE IS A SKIPED STEPS IT WILL EFFECT THE FALLING STEPS 
		if(steps == (NUMBER_OF_REV)-1)//IF THE STEPS EQUAL TO THE NUMBER OF RISING STEPS SO THIS MEANS NO SKIPED STEPS
			{
			Serial.println("THERE IS NO SKIPED STEPS");
			}
		else // ANY ELSE NUMBER OF THE STEPS V THIS MEANS THAT THERE IS A SKIPPED STEPS
			{
			Serial.println("THERE IS A SKIPED STEPS THE THE TOTAL FALLING STEPS %d",(steps+1));
			}
		steps=0;
        break;
    default:
        Serial.println("Communication error...");
        break;
    }



}
