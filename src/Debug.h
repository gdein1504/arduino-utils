#ifndef DEBUG_MACROS
#define DEBUG_MACROS

#include "Arduino.h"


#define DEBUG_FLAG_PRINT_TMS B10000000
#define DEBUG_FLAG_NEWLINE  B01000000

#define DEBUG_ASYNC_BUF_LEN 8
#define DEBUG_ASYNC_BUF_MAX_MASK 7
 

 #define PRINT_ASYNC(buf, ...)        buf.push(0, 0, __VA_ARGS__)
 #define PRINT_ASYNC_TMS(buf, ...)    buf.push(DEBUG_FLAG_PRINT_TMS, millis(), __VA_ARGS__)
 #define PRINTLN_ASYNC(buf, ...)      buf.push(DEBUG_FLAG_NEWLINE, 0, __VA_ARGS__)
 #define PRINTLN_ASYNC_TMS(buf, ...)  buf.push(DEBUG_FLAG_PRINT_TMS || DEBUG_FLAG_NEWLINE, millis(), __VA_ARGS__)

#ifdef DEBUG
 #define DEBUG_PRINT_ASYNC(buf, ...)        PRINT_ASYNC(buf, __VA_ARGS__)
 #define DEBUG_PRINT_ASYNC_TMS(buf, ...)    PRINT_ASYNC_TMS(buf, __VA_ARGS__)
 #define DEBUG_PRINTLN_ASYNC(buf, ...)      PRINTLN_ASYNC(buf, __VA_ARGS__)
 #define DEBUG_PRINTLN_ASYNC_TMS(buf, ...)  PRINTLN_ASYNC_TMS(buf, __VA_ARGS__)
 
 #define DEBUG_PRINT(...)       {Serial.print (__VA_ARGS__); Serial.flush();}
 #define DEBUG_PRINTLN(...)     {Serial.println(__VA_ARGS__); Serial.flush();}
 #define DEBUG_PRINT_TMS(...)   {Serial.print(millis()); Serial.print (": "); Serial.print (__VA_ARGS__); Serial.flush();}
 #define DEBUG_PRINTLN_TMS(...) {Serial.print(millis()); Serial.print (": "); Serial.println(__VA_ARGS__); Serial.flush();}
#else // debug undefined
 #define DEBUG_PRINT_ASYNC(...)
 #define DEBUG_PRINT_ASYNC_TMS(...)
 #define DEBUG_PRINTLN_ASYNC(...)
 #define DEBUG_PRINTLN_ASYNC_TMS(...)
 
 #define DEBUG_PRINT(...)    
 #define DEBUG_PRINTLN(...)  
 #define DEBUG_PRINT_TMS(...)
 #define DEBUG_PRINTLN_TMS(...)
#endif

#ifdef WARN
 #define WARN_PRINT_ASYNC(buf, ...)       PRINT_ASYNC(buf, __VA_ARGS__)
 #define WARN_PRINT_ASYNC_TMS(buf, ...)   PRINT_ASYNC_TMS(buf, __VA_ARGS__)
 #define WARN_PRINTLN_ASYNC(buf, ...)     PRINTLN_ASYNC(buf, __VA_ARGS__)
 #define WARN_PRINTLN_ASYNC_TMS(buf, ...) PRINTLN_ASYNC_TMS(buf, __VA_ARGS__)
 
 #define WARN_PRINT(...)        {Serial.print (__VA_ARGS__); Serial.flush();}
 #define WARN_PRINTLN(...)      {Serial.println(__VA_ARGS__); Serial.flush();}
 #define WARN_PRINT_TMS(...)    {Serial.print(millis()); Serial.print (": "); Serial.print (__VA_ARGS__); Serial.flush();}
 #define WARN_PRINTLN_TMS(...)  {Serial.print(millis()); Serial.print (": "); Serial.println(__VA_ARGS__); Serial.flush();}

#else // WARN undefined
 #define WARN_PRINT_ASYNC(...)
 #define WARN_PRINT_ASYNC_TMS(...)
 #define WARN_PRINTLN_ASYNC(...)
 #define WARN_PRINTLN_ASYNC_TMS(...)
 
 #define WARN_PRINT(...)
 #define WARN_PRINTLN(...)
 #define WARN_PRINT_TMS(...)
 #define WARN_PRINTLN_TMS(...)
#endif


class AsyncDebugBuffer
{
private:
  char const *message[DEBUG_ASYNC_BUF_LEN];
  unsigned int msgTMS[DEBUG_ASYNC_BUF_LEN];
  int msgInt[DEBUG_ASYNC_BUF_LEN];
  char msgFormat[DEBUG_ASYNC_BUF_LEN];
  volatile byte curIdx=0;
public:
  void flush();
  void push(byte tmsNlCode, unsigned int curMillis, char const *msg, byte format=0)
  {
    message[curIdx]=msg;
    msgTMS[curIdx] = curMillis;
    msgFormat[curIdx] = format | tmsNlCode;
    curIdx= (curIdx+1) & DEBUG_ASYNC_BUF_MAX_MASK;
  }
  void push(byte tmsNlCode, unsigned int curMillis, int num, byte format=DEC)
  {
    msgInt[curIdx] = num;
    msgTMS[curIdx] = curMillis;
    msgFormat[curIdx] = format | tmsNlCode;
    curIdx= (curIdx+1) & DEBUG_ASYNC_BUF_MAX_MASK;
  }

};


#endif
