#include "ScheduledFunction.h"
#include "PinTrigger.h"
#define DEBUG
#include "Debug.h"

#define ROTARY_A_PIN  3
#define ROTARY_B_PIN  4
#define LED_BLINK_PIN 5

int blinkInterval = 500; // wait time for blinks in millis
bool isBlinking = true;
bool blinkLedState = LOW;

ScheduledFunction blinker(blinkFunction, blinkInterval , true, true);
ScheduledFunction resetInt(resetInterval, 0, false, false);
PinTrigger trigger(PORTD_IDX, true, 2); // PORTD contains pins 0-7 

SET_INTERRUPTHANDLER_PORTD(trigger); // if we use pinchange interrupts, then this macro must be called to set the interrupt handler

void setup()
{
  Serial.begin(9600);
  DEBUG_PRINTLN(F("Blink starting..."));
  pinMode(ROTARY_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_B_PIN, INPUT_PULLUP);
  pinMode(LED_BLINK_PIN, INPUT_PULLUP);
  DEBUG_PRINTLN(F("Pinmodes set..."));

  trigger.setRotaryListener(blinkSpeedControl, ROTARY_A_PIN, ROTARY_B_PIN);
  trigger.setListener(ledBlinkBtn, LED_BLINK_PIN);
  DEBUG_PRINTLN("Listeners set, setup completed");
}

void loop() {
  trigger.process();

  uint32_t curMillis = millis();
  blinker.process(curMillis);
  resetInt.process(curMillis);
}

void ledBlinkBtn(byte pinValue, byte pinNum)
{
  if (pinValue == LOW) // button has been pressed
  {
    if(blinker.isActive())
      blinker.cancel();
    else
      blinker.reschedule(blinkInterval, true);
    isBlinking != isBlinking;
    resetInt.reschedule(2000);
  }
  else // button has been released
  {
    resetInt.cancel(); // if the button is pressed for less than 2s, then resetInt will not be called
  }
}

void blinkSpeedControl(int8_t delta)
{
  blinkInterval += delta*10;
  if(blinkInterval<10)
    blinkInterval = 10;
  blinker.reschedule(blinkInterval, true);
  DEBUG_PRINT(F("BlinkSpeed changed to [ms]: "));
  DEBUG_PRINTLN(blinkInterval);
}

void resetInterval()
{
  blinkInterval=500;
  blinker.reschedule(blinkInterval, true);
}

void blinkFunction()
{
  if(blinkLedState==HIGH)
    blinkLedState=LOW;
  else
    blinkLedState=HIGH;
  digitalWrite(LED_BUILTIN, blinkLedState);
}
