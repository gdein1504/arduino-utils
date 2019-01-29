#include "PinTrigger.h"
//#define DEBUG
#define WARN
#define PERF_TIME
#include "Debug.h"

PinTrigger Trigger = PinTrigger();
extern unsigned long timer0_millis; // in interrupts we access this variable directly - this is faster

#define swap(value) asm("swap %0" : "=r" (value) : "0" (value)) 

#ifndef NO_INTERRUPTS
ISR(PCINT0_vect) //handle pin change interrupt for D8 to D13 here
{
  byte sr = SREG;
#ifdef PERF_TIME
  unsigned int timing = micros();
#endif
  Trigger.processInterrupt(PINB, PORTB_IDX);
#ifdef PERF_TIME
  timing = micros() - timing;
  PRINT_ASYNC(" PORTB: ");
  PRINTLN_ASYNC_INT(timing, DEC);
#endif
  SREG = sr;
}
ISR(PCINT1_vect) //handle pin change interrupt for A0 to A7 here
{
  byte sr = SREG;
#ifdef PERF_TIME
  unsigned int timing = micros();
#endif
  Trigger.processInterrupt(PINC, PORTC_IDX);
#ifdef PERF_TIME
  timing = micros() - timing;
  PRINT_ASYNC(" PORTC: ");
  PRINTLN_ASYNC_INT(timing, DEC);
#endif
  SREG = sr;
}
ISR(PCINT2_vect) //handle pin change interrupt for D0 to D7 here
{
  byte sr = SREG;
#ifdef PERF_TIME
  unsigned int timing = micros();
#endif
  Trigger.processInterrupt(PIND, PORTD_IDX);
#ifdef PERF_TIME
  timing = micros() - timing;
  PRINT_ASYNC(" PORTD: ");
  PRINTLN_ASYNC_INT(timing, DEC);
#endif
  SREG = sr;
}
#endif // NO_INTERRUPTS


PinTrigger::PinTrigger()
{
  for(byte i=0; i<PORT_COUNT; i++)
  {
    prevDigitalPorts[i] = 0xFF; // assuming pullup resistors
    registeredPins[i] = 0x00;
    isRotaryEncoder[i] = 0x00;
  #ifdef NO_INTERRUPTS
    debouncedPins[i] = 0x00; // without interrupts we have to remember the debounce state in separate byte
  #endif
  }
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
    byte bitIdx = GET_BIT_IDX(pin);
    byte bitMsk = GET_BITMASK(bitIdx);
#ifndef NO_INTERRUPTS
    PCICR  |= GET_BITMASK(digitalPinToPCICRbit(pin)); // enable interrupt for the port of the pin
    *digitalPinToPCMSK(pin) |= bitMsk;  // enable pin change interrupt for the exact pin
#endif

    DEBUG_PRINT("PinTrigger::setListener for pin");
    DEBUG_PRINTDEC(pin);
    DEBUG_PRINT(", portIdx");
    DEBUG_PRINTDEC(port);
    DEBUG_PRINT(", bitIdx");
    DEBUG_PRINTDEC(bitIdx);
    DEBUG_PRINT(", PCMSK");
    DEBUG_PRINTLNBIN(*digitalPinToPCMSK(pin));

    listenerFP[listenerCount] = fp;
    lastTriggerTms[listenerCount] = 0;

    byte idx = listenerCount;
    if(deb)
      idx |= B01000000;  
    if(immediate)
      idx |= B10000000;  
    listenerIdx[port][bitIdx] = idx;
    registeredPins[port] |= bitMsk; 
    prevDigitalPorts[port] = GET_PINS(port); // clears all pending changes to avoid problems with setting pullups after initializing    
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
    byte port = GET_PORT_IDX(pinA);

    fpB |= 8; // indicates this is B
    fpB |= GET_BIT_IDX(pinA);
    fpA |= GET_BIT_IDX(pinB);
    isRotaryEncoder[port]|= GET_BITMASK(GET_BIT_IDX(pinA)) | GET_BITMASK(GET_BIT_IDX(pinB));
    
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
  
  byte PinTrigger::processInterrupt(byte port, byte portIdx)
  {
#ifdef NO_INTERRUPTS
    byte changedPins = (port ^ prevDigitalPorts[portIdx]) & registeredPins[portIdx] & ~debouncedPins[portIdx];
#else
    byte changedPins = (port ^ prevDigitalPorts[portIdx]) & *PortToPCMSK[portIdx]; // the PCMSK register already contains: registeredPins[portIdx] & ~debouncedPins[portIdx]
#endif
    DEBUG_PRINT_ASYNC("processInterrupt - pins: ");
    DEBUG_PRINT_ASYNC_INT(port, BIN);
    DEBUG_PRINT_ASYNC(", PCMSK");
    DEBUG_PRINT_ASYNC_INT(*PortToPCMSK[portIdx], BIN);
    DEBUG_PRINT_ASYNC(", portIdx");
    DEBUG_PRINT_ASYNC_INT(portIdx, DEC);
    DEBUG_PRINT_ASYNC(", prevPins");
    DEBUG_PRINTLN_ASYNC_INT(prevDigitalPorts[portIdx], BIN);

    if(changedPins==0) // if we rely on interrupts, this should actually not happen
    {
      DEBUG_PRINT_ASYNC("No Pin changed in processInterrupt");
      DEBUG_PRINT_ASYNC_INT(port, BIN);
      DEBUG_PRINT_ASYNC(", PCMSK");
      DEBUG_PRINT_ASYNC_INT(*PortToPCMSK[portIdx], BIN);
      DEBUG_PRINT_ASYNC(", prevPins");
      DEBUG_PRINTLN_ASYNC_INT(prevDigitalPorts[portIdx], BIN);
      return 0;
    }

    prevDigitalPorts[portIdx] ^= changedPins; // flip only valid changes - i.e. ignore changed debouncing pins

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
    byte idx = listenerIdx[portIdx][pin];
    byte idxNum = idx & B00111111;

    if(isRotaryEncoder[portIdx] & curPinMask)
    {
      byte *rotDef = (byte*)&(listenerFP[idxNum]);
      byte otherBitIdx = *rotDef & 0x7;
      byte otherPinValue = (port & GET_BITMASK(otherBitIdx)) != 0;
      byte *otherRotDef = (byte*)&(listenerFP[listenerIdx[portIdx][otherBitIdx] & B00111111]);

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
      if(idx & B01000000) // debounce
      {
#ifdef NO_INTERRUPTS
        debouncedPins[portIdx] |= curPinMask;
#else
        *PortToPCMSK[portIdx] &= ~curPinMask; // disable pin change interrupt for the debounced pin
#endif
        lastTriggerTms[idxNum] = (byte)timer0_millis;
        DEBUG_PRINT_ASYNC_TMS("debounce pin");
      }
      if(idx & B10000000) // call immediately, if the flag is set 
      {
        listenerFP[idxNum](pinValue, GET_PIN_NUM(portIdx, pin));
      }
      else if((queueLength & B01111111)< PINTRIGGER_QUEUE_LEN)
      {
        byte tempPort = portIdx;
        swap(tempPort);
        DEBUG_PRINT_ASYNC("queueing: ");
        DEBUG_PRINTLN_ASYNC_INT(tempPort | (pin<<1) | pinValue, BIN);
        callQueue[queueLength & B01111111] =  tempPort | (pin<<1) | pinValue; // queue the listener call for execution via process
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

  void PinTrigger::expireDebouncing(byte portIdx, byte curMillisLS8B)
  {
    byte msSinceInterrupt;
#ifdef NO_INTERRUPTS
    byte debouncedPins =  this->debouncedPins[portIdx];
#else
    byte debouncedPins =  registeredPins[portIdx] ^ *PortToPCMSK[portIdx];
#endif

    byte pin = FIRST1BIT(debouncedPins);
    byte debPinCount = BITCOUNT(debouncedPins);
    byte curPinMask = GET_BITMASK(pin);
    byte debPinsLeft = debPinCount;
    for(; debPinsLeft>0; pin++, curPinMask<<=1)
    {
      if(debouncedPins & curPinMask) // if true, its still debouncing, check if the debouncing has expired
      {
        debPinsLeft--;
        byte idxNum = listenerIdx[portIdx][pin] & B00111111;
        msSinceInterrupt = curMillisLS8B - lastTriggerTms[idxNum]; // this works even if one or both arguments have rolled over
        if(msSinceInterrupt >= DEBOUNCE_TIME)
        {
          DEBUG_PRINT_TMS("turn off debouncing: lastTriggerTms: ");
          DEBUG_PRINTDEC(lastTriggerTms[idxNum]);
          DEBUG_PRINT(", curMills8bit: ");
          DEBUG_PRINTDEC(curMillisLS8B);
          DEBUG_PRINT(", PCMSK: ");
          DEBUG_PRINTBIN(*PortToPCMSK[portIdx]);
          DEBUG_PRINT(", regPins: ");
          DEBUG_PRINTBIN(registeredPins[portIdx]);
          DEBUG_PRINT(", portIdx: ");
          DEBUG_PRINTDEC(portIdx);
          DEBUG_PRINT(", pin: ");
          DEBUG_PRINTLNDEC(pin);
#ifdef NO_INTERRUPTS
          this->debouncedPins[portIdx] &= !curPinMask; // debouncing expired, clear the debounce flag
#else
          *PortToPCMSK[portIdx] |= curPinMask;
#endif
          if((GET_PINS(portIdx) ^ prevDigitalPorts[portIdx]) & curPinMask) // pin changed after last interrupt
          {
            DEBUG_PRINTLN("simulate interrupt after debouncing");
            processInterrupt(prevDigitalPorts[portIdx] ^ curPinMask, portIdx); // we call the interrupt handler again
          }
#ifdef WARN
          if(msSinceInterrupt > (DEBOUNCE_TIME + 5))
          {
            WARN_PRINT_TMS("late debouncing turned off more than 5ms late: lastTriggerTms: ");
            WARN_PRINTDEC(lastTriggerTms[idxNum]);
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
    byte i;
    // first, reset any debounceFlags if the debounce time has expired
    for(i=0; i<PORT_COUNT; i++)
    {
#ifdef NO_INTERRUPTS
      if(debouncedPins[i]!=0)
#else
      if(registeredPins[i]!= *PortToPCMSK[i])
#endif
      {
        //DEBUG_PRINT("Debouncing Port ");
        //DEBUG_PRINTLNDEC(i);
        expireDebouncing(i, (byte)timer0_millis);
      }
#ifdef NO_INTERRUPTS
    if(GET_PINS(i)!=prevDigitalPorts[i])
    {
      processInterrupt(GET_PINS(i), i);
      lastTriggerTMS = millis();
    }
#endif
    }

    byte ints = queueLength & B01111111;

#ifndef NO_INTERRUPTS
    i=0;
    // now we process the queue to actually call the listeners:
processQueue:
    for(; i<ints; i++)
    {
      DEBUG_PRINTLN("PinTrigger::process int");

      byte curQueueEntry = callQueue[i]; 
      byte port = curQueueEntry; // LSB: high/low, next 3 bits: bitIdx, next 4 bits: portNum
      swap(port);
      port &= 0x0F;
      byte bitIdx = (curQueueEntry>>1) & B0000111;

      byte idxNum = listenerIdx[port][bitIdx] & B00111111;
      DEBUG_PRINT("PinTrigger::process about to call listener idx:");
      DEBUG_PRINTDEC(idxNum);
      DEBUG_PRINT(", queueEntry:");
      DEBUG_PRINTBIN(callQueue[i]);
      DEBUG_PRINT(", port:");
      DEBUG_PRINTDEC(port);
      DEBUG_PRINT(", bitIdx:");
      DEBUG_PRINTLNDEC(bitIdx);
      
      listenerFP[idxNum](curQueueEntry & 1, GET_PIN_NUM(port, bitIdx));
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
#endif
    DEBUG_PRINTLN("PinTrigger::process - listener calling complete");

    if(queueLength & B10000000)
    {
      queueLength &= B01111111;
    // process rotaryEncoders
      for(i=0; i<PORT_COUNT; i++)
      {
        byte isRot = isRotaryEncoder[i];
        if(isRot!=0)
        {
          byte bitIdx = FIRST1BIT(isRot);
          byte rotPinCount = BITCOUNT(isRot);
          byte curPinMask = GET_BITMASK(bitIdx);
          byte rotPinsLeft = rotPinCount;
          for(; rotPinsLeft>0; bitIdx++, curPinMask<<=1)
          {
            if(isRot & curPinMask)
            {
              rotPinsLeft--;
              byte listenerIdxA=listenerIdx[i][bitIdx]& B00111111;
              byte *rotADef = (byte[])&(listenerFP[listenerIdxA]);
              if((rotADef[0] & 8)==0) // is it really A?
              {
                byte otherBitIdx = rotADef[0]& 0x7;
                byte *rotBDef = (byte[])&(listenerFP[listenerIdx[i][otherBitIdx] & B00111111]);
  
                signed char delta = (rotBDef[0]>>5) - (rotADef[0]>>5);
                if(delta!=0)
                {
                  rotADef[0]&=B00011111;
                  rotBDef[0]&=B00011111;
                  DEBUG_PRINT("Process RotaryEncoder: fpA: ");
                  DEBUG_PRINTHEX((unsigned int)listenerFP[listenerIdxA]);
                  DEBUG_PRINT(", fpB: ");
                  DEBUG_PRINTHEX((unsigned int)listenerFP[listenerIdx[i][otherBitIdx] & B00111111]);
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
    }
    return ints;
  }


bool PinTrigger::safeToSleep()
{
#ifdef NO_INTERRUPTS // without interrupts, the uc will not wake up
  return false;
#else
  // if all registeredPins are high and nothing is currently debounced, we can go to sleep
  for(byte i=0; i<PORT_COUNT; i++)
  {
    if(*PortToPCMSK[i]!= registeredPins[i] || (prevDigitalPorts[i] & registeredPins[i]) != registeredPins[i])
      return false;
  }
  return true;
#endif
}

unsigned long PinTrigger::getLastTriggerTMS()
{
  return lastTriggerTMS;
}
