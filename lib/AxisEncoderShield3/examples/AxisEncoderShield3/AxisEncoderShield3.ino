// Robogaia.com
// Refactored by M. Kokko
// Modified 27-Mar-2025

#include "AxisEncoderShield3.h"


//*****************************************************
void setup() 
//*****************************************************
{
  Serial.begin(9600);
  initEncoderShield();
}

//*****************************************************
void loop() 
//*****************************************************
{
        long encoder1Value;
        long encoder2Value;
        long encoder3Value; 
        
        encoder1Value = getEncoderValue(1);  
        Serial.print("Encoder X= ");
        Serial.print(encoder1Value);
        
        encoder2Value = getEncoderValue(2);  
        Serial.print(" Encoder Y= ");
        Serial.print(encoder2Value);
        
        encoder3Value = getEncoderValue(3);  
        Serial.print(" Encoder Z= ");
        Serial.print(encoder3Value);
 
        Serial.print("\r\n");

     delay(100); 
 
}//end loop






