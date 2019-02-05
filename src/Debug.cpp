#include "Debug.h"
#include "Arduino.h"

void AsyncDebugBuffer::flush()
{
  for(int i=0; i<curIdx; i++)
  {
    byte fmt = msgFormat[i];
    if(fmt & DEBUG_FLAG_PRINT_TMS) 
    {
      Serial.print(msgTMS[i], DEC);
      Serial.print(": "); 
    }
    byte numFormat = fmt & B00111111;
    if(numFormat != 0) //its a number
    {
      if(fmt & DEBUG_FLAG_NEWLINE) 
        Serial.println(msgInt[i], numFormat);
      else
        Serial.print(msgInt[i], numFormat);
    }
    else // its a string
    {
      if(fmt & DEBUG_FLAG_NEWLINE)
        Serial.println(message[i]);
      else
        Serial.print(message[i]);
      message[i]=NULL;
    }
  }
  curIdx=0;
  Serial.flush();
}
