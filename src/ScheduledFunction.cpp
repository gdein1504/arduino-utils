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

#include "ScheduledFunction.h"


ScheduledFunction::ScheduledFunction(ScheduledFunctionP f, uint32_t execDelay, bool repeat = false, bool isActive = true)
{
  fp = f;
  this->active = isActive;
  if(repeat)
    this->repeatDelay = execDelay;
  else
    this->repeatDelay = 0;
  execTMS = millis() + execDelay;
}

void ScheduledFunction::reschedule(uint32_t execDelay, bool repeat = false)
{
  execTMS = millis() + execDelay;
  active = true;
  if(repeat)
    this->repeatDelay = execDelay;
  else
    this->repeatDelay = 0;
}

bool ScheduledFunction::isActive()
{
  return active;
}
void ScheduledFunction::cancel()
{
  active = false;
}


/**
 * returns true, if there is still a function pending
 */
bool ScheduledFunction::process(uint32_t curMillis = millis())
{
  if(!active)
  {
    return false;
  }

  if((curMillis - execTMS)<1000000) // to account for roll overs
  {
    if(repeatDelay!=0)
      execTMS = curMillis + repeatDelay;
    else
      active = false;
    fp(); // call the function
  }

  return active;
}
