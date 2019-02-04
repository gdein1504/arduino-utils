#ifndef PIN_TRIGGER
#define PIN_TRIGGER

#include "Arduino.h"

#if ( defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)  )
  volatile static byte* const PortToPCMSK[] = { &PCMSK2, &PCMSK0, &PCMSK1 };
#elif defined(__AVR_ATmega16__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega32__)
  #define NO_INTERRUPTS // these chips don't have pin change interrupts
#else
 #define NO_INTERRUPTS
#endif

#ifndef DEBOUNCE_TIME
  #define DEBOUNCE_TIME 20
#endif

#define PORTD_IDX 0
#define PORTB_IDX 1
#define PORTC_IDX 2

//handle pin change interrupt for D8 to D13 here 
#define SET_INTERRUPTHANDLER_PORTB(TRIGGER) \
ISR(PCINT0_vect) \
{ \
  TRIGGER.processInterrupt(PINB); \
}

//handle pin change interrupt for A0 to A7 here 
#define SET_INTERRUPTHANDLER_PORTC(TRIGGER) \
ISR(PCINT1_vect) \
{ \
  TRIGGER.processInterrupt(PINC); \
}

//handle pin change interrupt for D0 to D7 here:
#define SET_INTERRUPTHANDLER_PORTD(TRIGGER) \
ISR(PCINT2_vect) \
{ \
  TRIGGER.processInterrupt(PIND); \
}

volatile static byte* const PortToPins[] = { &PIND, &PINB, &PINC };

#define GET_PINS(port) (*PortToPins[port])

// port B has only 6 pins, so so for portc we have to subtract 2
#define GET_PORT_IDX(pin) ((pin + (pin<14?0:2))>>3)
#define GET_BIT_IDX(pin) ((pin + (pin<14?0:2)) & 7)
#define GET_PIN_NUM(port, bitIdx) (((port)<<3) + (bitIdx) - (((port)>1)<<1))

typedef void (* PinListenerFunctionPointer) (byte pinValue, byte pinNum);
typedef void (* RotaryListenerFunctionPointer) (signed char delta);

class PinTrigger
{
private:
  bool useInterrupts;
  byte portIdx;
  byte debouncedPins = 0; // only used when there are no interrupts;
  unsigned long lastTriggerTMS=0; // holds the TMS of very last trigger
  byte debounceFlags;
  byte immediateFlags;
  byte prevDigitalPorts = 0xFF;
  byte registeredPins = 0;
  byte isRotaryEncoder; //each bit holds a flag if this pin is connected to a rotary encoder

  PinListenerFunctionPointer listenerFP[8]; // holds the function pointer to the listener,
          // for rotary encoders it holds: MSByte: part of the function pointer, 
          // LSByte: lsbit: 3bit for the index of the other pin, 1 bit to indicate A/B:0 for A, 1 for B pin, 1 bit for state of a/b, 3 bit contain the delta on A pin positive, on B pin negative
  byte lastTriggerTms[8]; // holds the 8 least significant bits of the millis when this interrupt was last triggered
  byte listenerCount = 0;
  volatile byte queueLength = 0; // MSB indicates that a rotary encoder has turned
  byte *callQueue; // this queue is for calling interrupt handlers outside the interrupt scope to allow e.g. serial communication, delay etc.
                              // the byte contains the high/low flag in the LSB, then 3 bit for bit pin Idx
  byte callQueueSize;
  void expireDebouncing(byte curMillis8LSB);
  byte setLstnr(PinListenerFunctionPointer fp, byte pin, bool debounce, bool immediate);

public:
  PinTrigger(byte portIdx, bool useInterrupts, byte queueLength=4);
  byte processInterrupt(byte pins); // this has to be public because it is called from the interrupt routine, should not be called otherwise
  /**
   * Allows registering a rotary listener to two pins. NOTE: the two pins have to be on the same port!
   */
  byte setRotaryListener(RotaryListenerFunctionPointer fp, byte pinA, byte pinB);

  /**
   * Set a listener that is called whenever a pin change interrupt is triggered for a specific pin.
   * The listener is not called in the interrupt scope, so you can e.g. do serial communication, delay etc. directly in the listener
   */
  byte setListener(PinListenerFunctionPointer fp, byte pin, bool debounce=true);

  /**
   * Set a listener that is called immediately in the interrupt scope. So these eventhandlers cannot do e.g. serial communciation
   * or delay etc. 
   */
  byte setListenerImmediate(PinListenerFunctionPointer fp, byte pin, bool debounce=true); // this function does not queue the listeners, but calls them in the context of the interrupt

  /**
   * This function has to be called frequently in the loop. It takes care of executing the listeners and debouncing the interrupts
   */
  byte process();

  /**
   * This function returns true, if all registered pins are high (i.e. no button is currently pressed) and no pin is currently debounced.
   * If it returns true, the microcontroller can safely go to power down sleep (LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF) and will wake up 
   * on the next button press, or rotary turn
   */
  bool safeToSleep();

  /**
   * returns the TMS of the last Time a trigger has been called. Can be used to detect for how long there has been no user interaction
   */
  unsigned long getLastTriggerTMS();
};

#endif
