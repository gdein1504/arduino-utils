#include "ScheduledFunction.h"
#include "PinTrigger.h"
#include "Debug.h"

#define ROTARY_A_PIN  3
#define ROTARY_B_PIN  4

#define LED_ON_PIN    5
#define LED_OFF_PIN   6
#define LED_BLINK_PIN 7

unsigned int blinkWaitTime = 500; // wait time for blinks in millis
bool isBlinking = false;
bool blinkLedState = LOW;

ScheduledFunction blinker = ScheduledFunction(blinkFunction, blinkWaitTime , true, false);

void setup()
{
  pinMode(ROTARY_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_B_PIN, INPUT_PULLUP);
  pinMode(LED_ON_PIN, INPUT_PULLUP);
  pinMode(LED_BLINK_PIN, INPUT_PULLUP);

  registerRotaryEncoder(blinkSpeedControl, ROTARY_A_PIN, ROTARY_B_PIN);
  registerButton(ledOnBtn, LED_ON_PIN);
  registerButton(ledOffBtn, LED_OFF_PIN);
  registerButton(ledBlinkBtn, LED_BLINK_PIN);
}

void loop() {
  Trigger.process();
  blinker.process();
}

void ledOnBtn(byte pinValue, byte pinNum)
{
  if (pinValue == LOW) // button has been released
  {
    isBlinking = false;
    digitalWrite(LED_BUILTIN, HIGH);
  }
}
void ledOffBtn(byte pinValue, byte pinNum)
{
  if (pinValue == LOW) // button has been released
  {
    isBlinking = false;
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void ledBlinkBtn(byte pinValue, byte pinNum)
{
  if (pinValue == LOW) // button has been released
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
  blinker.reschedule(blinkWaitTime, true);
}

void blinkFunction()
{
  if(blinkLedState==HIGH)
    blinkLedState=LOW;
  else
    blinkLedState=HIGH;
  
  digitalWrite(LED_BUILTIN, blinkLedState);
}
