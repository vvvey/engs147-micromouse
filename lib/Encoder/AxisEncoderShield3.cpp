#include "AxisEncoderShield3.h"
//***************************************************** 
void initEncoderShield(void)
//*****************************************************
{
  pinMode(CHIP_SEL_PIN_1, OUTPUT);
  pinMode(CHIP_SEL_PIN_2, OUTPUT);
  pinMode(CHIP_SEL_PIN_3, OUTPUT);
  
  digitalWrite(CHIP_SEL_PIN_1, HIGH);
  digitalWrite(CHIP_SEL_PIN_2, HIGH);
  digitalWrite(CHIP_SEL_PIN_3, HIGH);
 
  LS7366_Init();
  delay(100); 

}//end func

//*****************************************************  
long getEncoderValue(int encoder)
//*****************************************************
{
    unsigned int count1Value, count2Value, count3Value, count4Value;
    long result;
    
    selectEncoder(encoder);
    
     SPI.transfer(0x60); // Request count
    count1Value = SPI.transfer(0x00); // Read highest order byte
    count2Value = SPI.transfer(0x00);
    count3Value = SPI.transfer(0x00);
    count4Value = SPI.transfer(0x00); // Read lowest order byte
    
    deselectEncoder(encoder);
   
    result= ((long)count1Value<<24) + ((long)count2Value<<16) + ((long)count3Value<<8) + (long)count4Value;
    
    return result;
}//end func

//*************************************************
void selectEncoder(int encoder)
//*************************************************
{
  switch(encoder)
  {
     case 1:
        digitalWrite(CHIP_SEL_PIN_1,LOW);
        break;
     case 2:
       digitalWrite(CHIP_SEL_PIN_2,LOW);
       break;
     case 3:
       digitalWrite(CHIP_SEL_PIN_3,LOW);
       break;    
  }//end switch
  
}//end func

//*************************************************
void deselectEncoder(int encoder)
//*************************************************
{
  switch(encoder)
  {
     case 1:
        digitalWrite(CHIP_SEL_PIN_1,HIGH);
        break;
     case 2:
       digitalWrite(CHIP_SEL_PIN_2,HIGH);
       break;
     case 3:
       digitalWrite(CHIP_SEL_PIN_3,HIGH);
       break;    
  }//end switch
  
}//end func



// LS7366 Initialization and configuration
//*************************************************
void LS7366_Init(void)
//*************************************************
{
       
    // SPI initialization
    SPI.begin();
    //SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
    delay(10);
   
   digitalWrite(CHIP_SEL_PIN_1,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(CHIP_SEL_PIN_1,HIGH); 
   
   
   digitalWrite(CHIP_SEL_PIN_2,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(CHIP_SEL_PIN_2,HIGH); 
   
   
   digitalWrite(CHIP_SEL_PIN_3,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(CHIP_SEL_PIN_3,HIGH); 
   
}//end func





