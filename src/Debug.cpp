#include "Debug.h"
#include "Arduino.h"

void AsyncDebugBuffer::flush()
{
  for(int i=0; i<curIdx; i++)
  {
    if(msgFormat[i] & DEBUG_FLAG_PRINT_TMS) 
    {
      Serial.print(msgTMS[i], DEC);
      Serial.print(": "); 
    }
    if(msgFormat[i]>3) //its a number
    {
      if(msgFormat[i]&DEBUG_FLAG_NEWLINE) 
        Serial.println(msgInt[i], msgFormat[i] & B00111111);
      else
        Serial.print(msgInt[i], msgFormat[i] & B00111111);
    }
    else // its a string
    {
      if(msgFormat[i]&DEBUG_FLAG_NEWLINE)
        Serial.println(message[i]);
      else
        Serial.print(message[i]);
      message[i]=NULL;
    }
  }
  curIdx=0;
  Serial.flush();
}
