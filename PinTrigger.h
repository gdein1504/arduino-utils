#include "Arduino.h"

//#if ( defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)  )
  volatile static byte* const PortToPCMSK[] = { &PCMSK2, &PCMSK0, &PCMSK1 };
//#elif defined(__AVR_ATmega16__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega32__)
 // #define NO_INTERRUPTS // these chips dont have pin change interrupts
//#else
// #define NO_INTERRUPTS
//#endif

#define registerButton(functionName, pin) Trigger.setListener(functionName, pin)
#define registerRotaryEncoder(functionName, pinA, pinB) Trigger.setRotaryListener(functionName, pinA, pinB)

#ifndef DEBOUNCE_TIME
  #define DEBOUNCE_TIME 20
#endif
  
#define PORT_COUNT 3
#define PORTD_IDX 0
#define PORTB_IDX 1
#define PORTC_IDX 2

volatile static byte* const PortToPins[] = { &PIND, &PINB, &PINC };

#define GET_PINS(port) (*PortToPins[port])

// port B has only 6 pins, so so for portc we have to subtract 2
#define GET_PORT_IDX(pin) ((pin + (pin<14?0:2))>>3)
#define GET_BIT_IDX(pin) ((pin + (pin<14?0:2)) & 7)
#define GET_PIN_NUM(port, bitIdx) (((port)<<3) + (bitIdx) - (((port)>1)<<1))

#ifndef MAX_LISTENERS
  #define MAX_LISTENERS 8
#endif

#ifndef PINTRIGGER_QLEN
  #ifdef NO_INTERRUPTS
    #define PINTRIGGER_QUEUE_LEN 0
  #else
    #define PINTRIGGER_QUEUE_LEN 4
  #endif
#endif

typedef void (* PinListenerFunctionPointer) (byte pinValue, byte pinNum);
typedef void (* RotaryListenerFunctionPointer) (signed char delta);

class PinTrigger
{
private:
#ifdef NO_INTERRUPTS
  byte debouncedPins[PORT_COUNT]; //= 0x0000;
#endif
  unsigned long lastTriggerTMS=0; // holds the TMS of very last trigger
  byte listenerIdx[PORT_COUNT][8]; //maps pin to the index of the listener 6LSB contain listenerIdx, MSB contains immediate Flag 2ndMSB contains debounce flag
  byte prevDigitalPorts[PORT_COUNT]; // = 0xFFFF;
  byte registeredPins[PORT_COUNT]; //= 0x0000;
  byte isRotaryEncoder[PORT_COUNT]; //holds a flag if this pin is connected to a rotary encoder

  PinListenerFunctionPointer listenerFP[MAX_LISTENERS]; // holds the function pointer to the listener,
          // for rotary encoders it holds: MSByte: part of the function pointer, 
          // LSByte: lsbit: 3bit for the index of the other pin, 1 bit to indicate A/B:0 for A, 1 for B pin, 1 bit for state of a/b, 3 bit contain the delta on A pin positive, on B pin negative
  byte lastTriggerTms[MAX_LISTENERS]; // holds the 8 least significant bits of the millis when this interrupt was last triggered
  byte listenerCount = 0;
  volatile byte queueLength = 0; // MSB indicates that a rotary encoder has turned
  byte callQueue[PINTRIGGER_QUEUE_LEN]; // this queue is for calling interrupt handlers outside the interrupt scope to allow e.g. serial communication, delay etc.
                              // the byte contains the high/low flag in the LSB, then 3 bit for bit Idx, 4 bit for port number

  void expireDebouncing(byte portIdx, byte curMillis8LSB);
  void handleRotaryEncoderA(byte pinValue, byte *aDef, byte *bDef, byte allPins);
  void handleRotaryEncoderB(byte pinValue, byte *aDef, byte *bDef, byte allPins);
  byte setLstnr(PinListenerFunctionPointer fp, byte pin, bool debounce, bool immediate);

public:
  PinTrigger();
  byte processInterrupt(byte pins, byte portIdx); // this has to be public because it is called from the interrupt routine, should not be called otherwise
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

extern PinTrigger Trigger;
