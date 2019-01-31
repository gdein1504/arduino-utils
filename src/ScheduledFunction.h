/***********************************USAGE*****************************

//execute a function call in 100ms:
uint32_t handle = Scheduler.scheduleCall(myFunction, 100);
// or for debugging purposes:
uint32_t handle = Scheduler.scheduleCall(myFunction, 100, "myFunction");

//cancel a scheduled call:
Scheduler.cancel(handle);

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
  Scheduler.scheduleCall(myFunction, 60000); // call again in 60s
}
*********************************************************************/


#ifndef SCHEDULEDFUNCTION
#define SCHEDULEDFUNCTION

typedef void (* ScheduledFunctionP) ();

class ScheduledFunction
{
private:
  ScheduledFunctionP fp;
  uint32_t execTMS;
  bool active = true;
  uint32_t repeatDelay;

public:
  ScheduledFunction(ScheduledFunctionP f, uint32_t callDelay, bool repeat=false, bool isActive=true);
  void reschedule(uint32_t execDelay, bool repeat=false);
  void cancel();
  bool isActive();
  bool process(uint32_t curMillis = millis());
};
#endif
