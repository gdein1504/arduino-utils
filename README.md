# arduino-utils
Highly optimized Arduino utility library for pin debouncing, rotary encoders, scheduled function execution and efficient debugging.

## PinTrigger
Register callbacks for button presses, or rotary encoders. Takes care of debouncing, by default uses pin-change interrupts, but can work without interrupts as well, if you need another library that needs pin-change-interrupts

## FunctionScheduler
Allows delayed execution of functions without using the Arduino delay function 

## Debug
Contains a number of debug macros for logging. It allows asynchronous logging, so you can log from interrupt routines.

