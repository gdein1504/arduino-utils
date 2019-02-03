#include <ScheduledFunction.h>
#include <PinTrigger.h>
#define DEBUG
#include <Debug.h>

#define ROTARY_A_PIN  3
#define ROTARY_B_PIN  4
#define LED_BLINK_PIN 5

int blinkWaitTime = 500; // wait time for blinks in millis
bool isBlinking = true;
bool blinkLedState = LOW;

ScheduledFunction blinker(blinkFunction, blinkWaitTime , true, true);
PinTrigger trigger(PORTD_IDX, true); // PORTD contains pins 0-7 

SET_INTERRUPTHANDLER_PORTD(trigger); // if we use pinchange interrupts, then this Macro must be called to set the interrupt handler

void setup()
{
  Serial.begin(9600);
  DEBUG_PRINTLN("Blink starting...");
  pinMode(ROTARY_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_B_PIN, INPUT_PULLUP);
  pinMode(LED_BLINK_PIN, INPUT_PULLUP);
  DEBUG_PRINTLN("Pinmodes set...");

  trigger.setRotaryListener(blinkSpeedControl, ROTARY_A_PIN, ROTARY_B_PIN);
  trigger.setListener(ledBlinkBtn, LED_BLINK_PIN);
  DEBUG_PRINTLN("Listeners set");
}

void loop() {
  trigger.process();
  blinker.process();
}


void ledBlinkBtn(byte pinValue, byte pinNum)
{
  if (pinValue == LOW) // button has been pressed
  {
    if(blinker.isActive())
      blinker.cancel();
    else
      blinker.reschedule(blinkWaitTime, true);
    isBlinking != isBlinking;
  }
}


void blinkSpeedControl(int8_t delta)
{
  blinkWaitTime += delta*10;
  if(blinkWaitTime<10)
    blinkWaitTime = 10;
  blinker.reschedule(blinkWaitTime, true);
  DEBUG_PRINT("BlinkSpeed changed to ");
  DEBUG_PRINTDEC(blinkWaitTime);
  DEBUG_PRINTLN("ms");
}

void blinkFunction()
{
  if(blinkLedState==HIGH)
    blinkLedState=LOW;
  else
    blinkLedState=HIGH;
  digitalWrite(LED_BUILTIN, blinkLedState);
}
