#include "Debug.h"
#include "Arduino.h"

#define DEBUG 
//#undef DEBUG
volatile byte DEBUG_MSG_IDX = 0;

//#ifdef DEBUG
char const *DEBUG_MSG[DEBUG_ASYNC_BUF_LEN];
unsigned int DEBUG_MSG_TMS[DEBUG_ASYNC_BUF_LEN];
int DEBUG_MSG_INT[DEBUG_ASYNC_BUF_LEN];
char DEBUG_MSG_FORMAT[DEBUG_ASYNC_BUF_LEN];
//#else
//char const *DEBUG_MSG[0];
//int DEBUG_MSG_INT[0];
//char DEBUG_MSG_FORMAT[0];
//#endif

void printDebugMsgAsync()
{
  for(int i=0; i<DEBUG_MSG_IDX; i++)
  {
    if(DEBUG_MSG_FORMAT[i]&2)
    {
      Serial.print(DEBUG_MSG_TMS[i], DEC);
      Serial.print(": "); 
    }
    if(DEBUG_MSG_FORMAT[i]>3) //its a number
    {
      if(DEBUG_MSG_FORMAT[i]&1)
        Serial.println(DEBUG_MSG_INT[i], DEBUG_MSG_FORMAT[i]>>2);
      else
        Serial.print(DEBUG_MSG_INT[i], DEBUG_MSG_FORMAT[i]>>2);
    }
    else // its a string
    {
      if(DEBUG_MSG_FORMAT[i]&1)
        Serial.println(DEBUG_MSG[i]);
      else
        Serial.print(DEBUG_MSG[i]);
      DEBUG_MSG[i]=NULL;
    }
  }
  Serial.flush();
  DEBUG_MSG_IDX=0;
}

