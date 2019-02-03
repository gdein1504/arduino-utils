#include "PinTrigger.h"
//#define DEBUG
//#define WARN
#include "Debug.h"
#include "utility/sharedutils.h"

extern unsigned long timer0_millis; // in interrupts we access this variable directly - this is faster

PinTrigger::PinTrigger(byte portIndex, bool useInterrupts)
{
  portIdx = portIndex;
  this->useInterrupts = useInterrupts;
}

byte PinTrigger::setListenerImmediate(PinListenerFunctionPointer fp, byte pin, bool deb)
{
  return setLstnr(fp, pin, deb, true);
}
byte PinTrigger::setListener(PinListenerFunctionPointer fp, byte pin, bool deb)
{
  return setLstnr(fp, pin, deb, false);
}

byte PinTrigger::setLstnr(PinListenerFunctionPointer fp, byte pin, bool deb, bool immediate)
{
//    ASSERT(listenerCount<MAX_LISTENERS-1, "Maximum number of listeners exceeded, use #define MAX_LISTENERS to increase the limit");
  byte port = GET_PORT_IDX(pin);
  if(port!=portIdx)
  {
    Serial.println("ERROR PIN of Listener not on the correct Port");
  }
  byte bitIdx = GET_BIT_IDX(pin);
  byte bitMsk = GET_BITMASK(bitIdx);
  if(useInterrupts)
  {
    PCICR  |= GET_BITMASK(digitalPinToPCICRbit(pin)); // enable interrupt for the port of the pin
    *digitalPinToPCMSK(pin) |= bitMsk;  // enable pin change interrupt for the exact pin
  }
  else
    immediate=true;
    
  DEBUG_PRINT("PinTrigger::setListener for pin");
  DEBUG_PRINTDEC(pin);
  DEBUG_PRINT(", portIdx");
  DEBUG_PRINTDEC(portIdx);
  DEBUG_PRINT(", bitIdx");
  DEBUG_PRINTDEC(bitIdx);
  DEBUG_PRINT(", PCMSK");
  DEBUG_PRINTLNBIN(*digitalPinToPCMSK(pin));

  listenerFP[bitIdx] = fp;
  lastTriggerTms[bitIdx] = 0;

  if(deb)
    debounceFlags |= bitMsk;  
  if(immediate)
    immediateFlags |= bitMsk;  

  registeredPins |= bitMsk; 
  prevDigitalPorts = GET_PINS(port); // clears all pending changes to avoid problems with setting pullups after initializing    
  return listenerCount++;
}

byte PinTrigger::setRotaryListener(RotaryListenerFunctionPointer fp, byte pinA, byte pinB)
{
  DEBUG_PRINT("Adding rotaryEncoder pinA:");
  DEBUG_PRINTDEC(pinA);
  DEBUG_PRINT(", pinB:");
  DEBUG_PRINTDEC(pinB);

  unsigned int fpParts = (unsigned int)fp;
  unsigned int fpA = (unsigned int)fp & 0xFF00;
  unsigned int fpB = (unsigned int)fp << 8;

  fpB |= 8; // indicates this is B
  fpB |= GET_BIT_IDX(pinA);
  fpA |= GET_BIT_IDX(pinB);
  isRotaryEncoder|= GET_BITMASK(GET_BIT_IDX(pinA)) | GET_BITMASK(GET_BIT_IDX(pinB));
  
  DEBUG_PRINT(", pointer:");
  DEBUG_PRINTHEX(fpParts);
  DEBUG_PRINT(", fpA:");
  DEBUG_PRINTHEX(fpA);
  DEBUG_PRINT(", fpB:");
  DEBUG_PRINTHEX(fpB);
  DEBUG_PRINTLN("");

  setListenerImmediate(fpA, pinA, 0);
  return setListenerImmediate(fpB, pinB, 0);
}

byte PinTrigger::processInterrupt(byte port)
{
  byte changedPins;
  if(useInterrupts)
    changedPins = (port ^ prevDigitalPorts) & *PortToPCMSK[portIdx]; // the PCMSK register already contains: registeredPins[portIdx] & ~debouncedPins[portIdx]
  else
    changedPins = (port ^ prevDigitalPorts) & registeredPins & ~debouncedPins;
  
  DEBUG_PRINT_ASYNC("processInterrupt - pins: ");
  DEBUG_PRINT_ASYNC_INT(port, BIN);
  DEBUG_PRINT_ASYNC(", PCMSK");
  DEBUG_PRINT_ASYNC_INT(*PortToPCMSK, BIN);
  DEBUG_PRINT_ASYNC(", portIdx");
  DEBUG_PRINT_ASYNC_INT(portIdx, DEC);
  DEBUG_PRINT_ASYNC(", prevPins");
  DEBUG_PRINTLN_ASYNC_INT(prevDigitalPorts, BIN);

  if(changedPins==0) // if we rely on interrupts, this should actually not happen
  {
    DEBUG_PRINT_ASYNC("No Pin changed in processInterrupt");
    DEBUG_PRINT_ASYNC_INT(port, BIN);
    DEBUG_PRINT_ASYNC(", PCMSK");
    DEBUG_PRINT_ASYNC_INT(*PortToPCMSK, BIN);
    DEBUG_PRINT_ASYNC(", prevPins");
    DEBUG_PRINTLN_ASYNC_INT(prevDigitalPorts, BIN);
    return 0;
  }

  prevDigitalPorts ^= changedPins; // flip only valid changes - i.e. ignore changed debouncing pins

  byte pin, changedPinCount, curPinMask, changesLeft;
  changedPinCount = 1;

processIntLoop: // with a goto we can save a few comparisons here
//    pin = FIRST1BIT(changedPins);
  // the code below should be a bit faster than just using the FIRST1BIT macro
  byte temp = changedPins;
  if((temp & 0xF) == 0)
  {
    swap(temp);
    pin = first1BitperNib[temp & 0xF] + 4;
  }
  else
    pin = first1BitperNib[temp & 0xF];
  curPinMask = GET_BITMASK(pin);

  byte pinValue = (port & curPinMask) ? HIGH : LOW;
  DEBUG_PRINT_ASYNC_TMS("Interrupt: ");
  DEBUG_PRINTLN_ASYNC_INT(pin, DEC);

  if(isRotaryEncoder & curPinMask)
  {
    byte *rotDef = (byte*)&(listenerFP[pin]);
    byte otherBitIdx = *rotDef & 0x7;
    byte otherPinValue = (port & GET_BITMASK(otherBitIdx)) != 0;
    byte *otherRotDef = (byte*)&(listenerFP[otherBitIdx]);

    if(pinValue && otherPinValue && (*rotDef & B00010000)) // both pins at detent and we expect detent on this pin change
    {
      *rotDef&=B11101111; // reset the state
      *otherRotDef&=B11101111; 
      *rotDef+=B00100000; // increment counter by 1
      queueLength|=B10000000; // indicates rotary has been turned
    }
    else if (pinValue && !otherPinValue) // only the current pin is HIGH
      *otherRotDef|=B00010000; // expecting the other pin to go high next
  }
  else
  {
    if(debounceFlags & curPinMask) // debounce
    {
      if(useInterrupts)
        *PortToPCMSK[portIdx] &= ~curPinMask; // disable pin change interrupt for the debounced pin
      else
        debouncedPins |= curPinMask;

      lastTriggerTms[pin] = (byte)timer0_millis;
      DEBUG_PRINT_ASYNC_TMS("debounce pin");
    }
    if(immediateFlags & curPinMask) // call immediately, if the flag is set 
    {
      listenerFP[pin](pinValue, GET_PIN_NUM(portIdx, pin));
    }
    else if((queueLength & B01111111)< PINTRIGGER_QUEUE_LEN)
    {
      DEBUG_PRINT_ASYNC("queueing: ");
      DEBUG_PRINTLN_ASYNC_INT((pin<<1) | pinValue, BIN);
      callQueue[queueLength & B01111111] =  (pin<<1) | pinValue; // queue the listener call for execution via process
      queueLength++;
    }
    else
    {
      WARN_PRINTLN_ASYNC("ERROR: PinTrigger:callqueueOverflow - skipping interrupt");
    }
  }
  changedPins &= ~curPinMask;
  if(changedPins!=0)
  {
    changedPinCount++;
    goto processIntLoop;
  }
  
  return changedPinCount;
}

void PinTrigger::expireDebouncing(byte curMillisLS8B)
  {
    byte msSinceInterrupt;
    byte debouncedPins;
    if(useInterrupts)
      debouncedPins =  registeredPins ^ *PortToPCMSK[portIdx];
    else
      debouncedPins =  this->debouncedPins;

    byte pin = FIRST1BIT(debouncedPins);
    byte debPinCount = BITCOUNT(debouncedPins);
    byte curPinMask = GET_BITMASK(pin);
    byte debPinsLeft = debPinCount;
    for(; debPinsLeft>0; pin++, curPinMask<<=1)
    {
      if(debouncedPins & curPinMask) // if true, its still debouncing, check if the debouncing has expired
      {
        debPinsLeft--;
        msSinceInterrupt = curMillisLS8B - lastTriggerTms[pin]; // this works even if one or both arguments have rolled over
        if(msSinceInterrupt >= DEBOUNCE_TIME)
        {
          DEBUG_PRINT_TMS("turn off debouncing: lastTriggerTms: ");
          DEBUG_PRINTDEC(lastTriggerTms[pin]);
          DEBUG_PRINT(", curMills8bit: ");
          DEBUG_PRINTDEC(curMillisLS8B);
          DEBUG_PRINT(", PCMSK: ");
          DEBUG_PRINTBIN(*PortToPCMSK[portIdx]);
          DEBUG_PRINT(", regPins: ");
          DEBUG_PRINTBIN(registeredPins);
          DEBUG_PRINT(", portIdx: ");
          DEBUG_PRINTDEC(portIdx);
          DEBUG_PRINT(", pin: ");
          DEBUG_PRINTLNDEC(pin);
          if(useInterrupts)
            *PortToPCMSK[portIdx] |= curPinMask;
          else
            this->debouncedPins &= !curPinMask; // debouncing expired, clear the debounce flag

          if((GET_PINS(portIdx) ^ prevDigitalPorts) & curPinMask) // pin changed after last interrupt
          {
            DEBUG_PRINTLN("simulate interrupt after debouncing");
            processInterrupt(prevDigitalPorts ^ curPinMask); // we call the interrupt handler again
          }
#ifdef WARN
          if(msSinceInterrupt > (DEBOUNCE_TIME + 5))
          {
            WARN_PRINT_TMS("late debouncing turned off more than 5ms late: lastTriggerTms: ");
            WARN_PRINTDEC(lastTriggerTms[pin]);
            WARN_PRINT(", curMills8bit: ");
            WARN_PRINTDEC(curMillisLS8B);
            WARN_PRINT(", pin: ");
            WARN_PRINTDEC(pin);
            WARN_PRINT(", msSinceInterrupt: ");
            WARN_PRINTLNDEC(msSinceInterrupt);
          }
#endif
        }
      }
    }
  }


byte PinTrigger::process()
{
  // first, reset any debounceFlags if the debounce time has expired
  bool cond;
  if(useInterrupts)
    cond = registeredPins!= *PortToPCMSK[portIdx];
  else
    cond = debouncedPins!=0;
  if(cond)
  {
    //DEBUG_PRINT("Debouncing Port ");
    //DEBUG_PRINTLNDEC(i);
    expireDebouncing((byte)timer0_millis);
  }
  if(!useInterrupts && GET_PINS(portIdx)!=prevDigitalPorts)
  {
    processInterrupt(GET_PINS(portIdx));
    lastTriggerTMS = millis();
  }

  byte ints = queueLength & B01111111;
  if(useInterrupts) // without interrupts, listeners are called immediately
  {
    // now we process the queue to actually call the listeners:
processQueue:
    int i=0;
    for(i; i<ints; i++)
    {
      DEBUG_PRINTLN("PinTrigger::process int");

      byte curQueueEntry = callQueue[i]; 
      byte bitIdx = (curQueueEntry>>1) & B0000111;

      DEBUG_PRINT("PinTrigger::process about to call listener idx:");
      DEBUG_PRINTDEC(bitIdx);
      DEBUG_PRINT(", queueEntry:");
      DEBUG_PRINTBIN(callQueue[i]);
      DEBUG_PRINT(", port:");
      DEBUG_PRINTDEC(portIdx);
      DEBUG_PRINT(", bitIdx:");
      DEBUG_PRINTLNDEC(bitIdx);
      
      listenerFP[bitIdx](curQueueEntry & 1, GET_PIN_NUM(portIdx, bitIdx));
    }
    cli(); //disable interrupts
    if(i>0) // lastTriggerTMS is set in the process loop to not delay the interrupt handler
      lastTriggerTMS = timer0_millis;
    
    if((queueLength & B01111111)>i) //an interrupt happened after the loop has completed and before the interrupts have been disabled
    {
      DEBUG_PRINTLN_TMS("RaceCondition:handle more interrupts");
      sei();
      ints=queueLength & B01111111;
      goto processQueue;
    }
    queueLength&=B10000000;
    sei(); // reenable interrupts
  }
//  DEBUG_PRINTLN("PinTrigger::process - listener calling complete");

  if(queueLength & B10000000)
  {
    queueLength &= B01111111;
  // process rotaryEncoders
    if(isRotaryEncoder!=0)
    {
      byte bitIdx = FIRST1BIT(isRotaryEncoder);
      byte rotPinCount = BITCOUNT(isRotaryEncoder);
      byte curPinMask = GET_BITMASK(bitIdx);
      byte rotPinsLeft = rotPinCount;
      for(; rotPinsLeft>0; bitIdx++, curPinMask<<=1)
      {
        if(isRotaryEncoder & curPinMask)
        {
          rotPinsLeft--;
          byte *rotADef = (byte[])&(listenerFP[bitIdx]);
          if((rotADef[0] & 8)==0) // is it really A?
          {
            byte otherBitIdx = rotADef[0]& 0x7;
            byte *rotBDef = (byte[])&(listenerFP[otherBitIdx]);

            signed char delta = (rotBDef[0]>>5) - (rotADef[0]>>5);
            if(delta!=0)
            {
              rotADef[0]&=B00011111;
              rotBDef[0]&=B00011111;
              DEBUG_PRINT("Process RotaryEncoder: fpA: ");
              DEBUG_PRINTHEX((unsigned int)listenerFP[bitIdx]);
              DEBUG_PRINT(", fpB: ");
              DEBUG_PRINTHEX((unsigned int)listenerFP[otherBitIdx]);
              DEBUG_PRINT(", BbitIdx: ");
              DEBUG_PRINTDEC(otherBitIdx);
              DEBUG_PRINT(", DELTA: ");
              DEBUG_PRINTDEC(delta);
              unsigned int fPointer = (((unsigned int)rotADef[1])<<8) | rotBDef[1];
              DEBUG_PRINT(" Calling rotaryListener:");
              DEBUG_PRINTLNHEX(fPointer);
              
              ((RotaryListenerFunctionPointer)fPointer)(delta);
            }
          }
        }
      }
    }
  }
  return ints;
}


bool PinTrigger::safeToSleep()
{
  if(useInterrupts)  // without interrupts, the uc will not wake up
    return !(*PortToPCMSK[portIdx]!= registeredPins || (prevDigitalPorts & registeredPins) != registeredPins);
  else
    return false;
}

unsigned long PinTrigger::getLastTriggerTMS()
{
  return lastTriggerTMS;
}
