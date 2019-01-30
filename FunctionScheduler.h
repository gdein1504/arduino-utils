/***********************************USAGE*****************************

//execute a function call in 100ms:
uint32_t handle = scheduler.scheduleCall(myFunction, 100);
// or for debugging purposes:
uint32_t handle = scheduler.scheduleCall(myFunction, 100, "myFunction");

//cancel a scheduled call:
scheduler.cancel(handle);

//in the loop you need to make the following call frequently:
scheduler.process();

// to call a function every minute, call a function like this in the setup function:
void setup()
{
  myFunction();
}

void myFunction()
{
   // do stuff
  scheduler.scheduleCall(myFunction, 60000); // call again in 60s
}

*********************************************************************/


#include "sharedutils.h"

#ifndef FUNCTION_SCHEDULER
#define FUNCTION_SCHEDULER
#include "Arduino.h"


//#define DEBUG
#include "debug.h"

typedef void (* ScheduledFunction) ();

class FunctionScheduler
{
private:
  ScheduledFunction fpArray[8];
  uint32_t execTMS[8];
  uint8_t activeSlots = 0; // each bit holds the status 
#ifdef DEBUG
  char const *funcName[];
#endif
public:
  uint32_t scheduleCall(ScheduledFunction f, uint32_t execDelay, const char *functionName = "")
  {
    DEBUG_PRINT_TMS("scheduling functionCall to ");
    DEBUG_PRINT(functionName);
    DEBUG_PRINT(" in ms");
    DEBUG_PRINTLNDEC(execDelay);

    int8_t nextSlot = -1; //FIRST1BIT(^activeSlots);
    if(nextSlot==-1)
      return -1;

    activeSlots |= 1<<nextSlot;
    fpArray[nextSlot]=f;
#ifdef DEBUG
    funcName[nextSlot]=functionName;
#endif
    uint32_t t= millis() + execDelay;
    execTMS[nextSlot]= t;
    return (t<<8) | nextSlot;
  }
  
  void cancelCall(uint32_t handle)
  {
    uint8_t idx = handle; // lowest byte is the index
    if(idx>=0 && idx<8 && (execTMS[idx]<<8 | idx) == handle)
    {
      DEBUG_PRINT_TMS("cancelling functionCall to ");
      DEBUG_PRINTLN(funcName[idx]);
      activeSlots &= ~(1<<idx);
    }
    else
    {
      DEBUG_PRINT_TMS("invalid handle for scheduler.cancelCall: ");
      DEBUG_PRINTLNHEX(handle);
    }
  }
  
  
  /**
   * returns true, if there is still a function pending
   */
  bool process(uint32_t curMillis = millis())
  {
    if(activeSlots==0)
    {
      return false;
    }

    byte curSlot = FIRST1BITX(activeSlots);
    byte curMask = 1<<curSlot;
    
    for(; curMask<=activeSlots && curSlot<8; curSlot++, curMask<<=1)
    {
      if(activeSlots & curMask)
      {
        if((curMillis - execTMS[curSlot])<1000000) // to account for roll overs
        {
          DEBUG_PRINT_TMS("Scheduler: calling scheduled function: ");
          DEBUG_PRINTLN(funcName[curSlot]);
          ScheduledFunction f = fpArray[curSlot];
          activeSlots ^= curMask; // flip the activeSlot bit before calling the function
          f(); // call the function
        }
      }
    }
    return activeSlots!=0;
  }
};

FunctionScheduler Scheduler;

#endif
