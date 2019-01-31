#ifndef DEBUG_MACROS
#define DEBUG_MACROS

#include "Arduino.h"

extern char const *DEBUG_MSG[];
extern int DEBUG_MSG_INT[];
extern unsigned int DEBUG_MSG_TMS[];
extern char DEBUG_MSG_FORMAT[];
extern volatile byte DEBUG_MSG_IDX;

void printDebugMsgAsync();

 #define DEBUG_ASYNC_BUF_LEN 8
 #define PRINT_ASYNC(x) {DEBUG_MSG[DEBUG_MSG_IDX]=x;DEBUG_MSG_FORMAT[DEBUG_MSG_IDX]=0;DEBUG_MSG_IDX<DEBUG_ASYNC_BUF_LEN-1?DEBUG_MSG_IDX++:DEBUG_MSG_IDX;}
 #define PRINT_ASYNC_TMS(x) {DEBUG_MSG[DEBUG_MSG_IDX]=x;DEBUG_MSG_FORMAT[DEBUG_MSG_IDX]=2;DEBUG_MSG_TMS[DEBUG_MSG_IDX]=millis();DEBUG_MSG_IDX<DEBUG_ASYNC_BUF_LEN-1?DEBUG_MSG_IDX++:DEBUG_MSG_IDX;}
 #define PRINT_ASYNC_INT(x, f) {DEBUG_MSG_FORMAT[DEBUG_MSG_IDX]=f<<2; DEBUG_MSG_INT[DEBUG_MSG_IDX]=x;DEBUG_MSG_IDX<DEBUG_ASYNC_BUF_LEN-1?DEBUG_MSG_IDX++:DEBUG_MSG_IDX;}
 #define PRINTLN_ASYNC(x) {DEBUG_MSG[DEBUG_MSG_IDX]=x; DEBUG_MSG_FORMAT[DEBUG_MSG_IDX]=1; DEBUG_MSG_IDX<DEBUG_ASYNC_BUF_LEN-1?DEBUG_MSG_IDX++:DEBUG_MSG_IDX;}
 #define PRINTLN_ASYNC_TMS(x) {DEBUG_MSG[DEBUG_MSG_IDX]=x; DEBUG_MSG_FORMAT[DEBUG_MSG_IDX]=3; DEBUG_MSG_TMS[DEBUG_MSG_IDX]=millis();DEBUG_MSG_IDX<DEBUG_ASYNC_BUF_LEN-1?DEBUG_MSG_IDX++:DEBUG_MSG_IDX;}
 #define PRINTLN_ASYNC_INT(x, f) {DEBUG_MSG_FORMAT[DEBUG_MSG_IDX]=(f<<2)|1; DEBUG_MSG_INT[DEBUG_MSG_IDX]=x;DEBUG_MSG_IDX<DEBUG_ASYNC_BUF_LEN-1?DEBUG_MSG_IDX++:DEBUG_MSG_IDX;}
 #define PRINT_ASYNC_DEBUG_MSGS printDebugMsgAsync();

#ifdef DEBUG
 #define DEBUG_PRINT_ASYNC(x) PRINT_ASYNC(x)
 #define DEBUG_PRINT_ASYNC_TMS(x) PRINT_ASYNC_TMS(x)
 #define DEBUG_PRINT_ASYNC_INT(x, f) PRINT_ASYNC_INT(x, f)
 #define DEBUG_PRINTLN_ASYNC(x) PRINTLN_ASYNC(x)
 #define DEBUG_PRINTLN_ASYNC_TMS(x) PRINT_ASYNC_TMS(x)
 #define DEBUG_PRINTLN_ASYNC_INT(x, f) PRINTLN_ASYNC_INT(x, f)
 
 #define DEBUG_PRINT(x)    {Serial.print (x); Serial.flush();}
 #define DEBUG_PRINTLN(x)  {Serial.println(x); Serial.flush();}
 #define DEBUG_PRINT_TMS(x)    {Serial.print(millis()); Serial.print (": "); Serial.print (F(x)); Serial.flush();}
 #define DEBUG_PRINTLN_TMS(x)  {Serial.print(millis()); Serial.print (": "); Serial.println(F(x)); Serial.flush();}
 #define DEBUG_PRINTHEX(x)    {Serial.print(x, HEX); Serial.flush();}
 #define DEBUG_PRINTDEC(x) {Serial.print (x, DEC); Serial.flush();}
 #define DEBUG_PRINTBIN(x) {Serial.print (x, BIN); Serial.flush();}
 #define DEBUG_PRINTLNDEC(x) {Serial.println(x, DEC); Serial.flush();}
 #define DEBUG_PRINTLNBIN(x) {Serial.println(x, BIN); Serial.flush();}
 #define DEBUG_PRINTLNHEX(x) {Serial.println(x, HEX); Serial.flush();}
#else // debug undefined
 #define DEBUG_PRINT_ASYNC(x)
 #define DEBUG_PRINT_ASYNC_TMS(x)
 #define DEBUG_PRINT_ASYNC_INT(x, f)
 #define DEBUG_PRINTLN_ASYNC(x) 
 #define DEBUG_PRINTLN_ASYNC_TMS(x) 
 #define DEBUG_PRINTLN_ASYNC_INT(x, f) 
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINT_TMS(x)
 #define DEBUG_PRINTLN_TMS(x)
 #define DEBUG_PRINTBIN(x)
 #define DEBUG_PRINTHEX(x)
 #define DEBUG_PRINTLNDEC(x)
 #define DEBUG_PRINTLNBIN(x)
 #define DEBUG_PRINTLNHEX(x)
 #define DEBUG_PRINTLN(x)
#endif

#ifdef WARN
 #define WARN_PRINT_ASYNC(x) PRINT_ASYNC(x)
 #define WARN_PRINT_ASYNC_TMS(x) PRINT_ASYNC_TMS(x)
 #define WARN_PRINT_ASYNC_INT(x, f) PRINT_ASYNC_INT(x, f)
 #define WARN_PRINTLN_ASYNC(x) PRINTLN_ASYNC(x)
 #define WARN_PRINTLN_ASYNC_TMS(x) PRINT_ASYNC_TMS(x)
 #define WARN_PRINTLN_ASYNC_INT(x, f) PRINTLN_ASYNC_INT(x, f)
 
 #define WARN_PRINT(x)    {Serial.print(F(x)); Serial.flush();}
 #define WARN_PRINTLN(x)  {Serial.println(F(x)); Serial.flush();}
 #define WARN_PRINT_TMS(x)    {Serial.print(millis()); Serial.print (": "); Serial.print(F(x)); Serial.flush();}
 #define WARN_PRINTLN_TMS(x)  {Serial.print(millis()); Serial.print (": "); Serial.println(F(x)); Serial.flush();}
 #define WARN_PRINTDEC(x) {Serial.print (x, DEC); Serial.flush();}
 #define WARN_PRINTBIN(x) {Serial.print (x, BIN); Serial.flush();}
 #define WARN_PRINTLNDEC(x) {Serial.println(x, DEC); Serial.flush();}
 #define WARN_PRINTLNBIN(x) {Serial.println(x, BIN); Serial.flush();}
#else // WARN undefined
 #define WARN_PRINT_ASYNC(x) 
 #define WARN_PRINT_ASYNC_TMS(x) 
 #define WARN_PRINT_ASYNC_INT(x, f) 
 #define WARN_PRINTLN_ASYNC(x) 
 #define WARN_PRINTLN_ASYNC_TMS(x) 
 #define WARN_PRINTLN_ASYNC_INT(x, f) 
 #define WARN_PRINT(x)
 #define WARN_PRINTDEC(x)
 #define WARN_PRINTBIN(x)
 #define WARN_PRINT_TMS(x)
 #define WARN_PRINTLN_TMS(x)
 #define WARN_PRINTLNDEC(x)
 #define WARN_PRINTLNBIN(x)
 #define WARN_PRINTLN(x)
#endif

#endif

