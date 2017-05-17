#include "IRremote.h"

#define IR_INPUT_PIN A2
#define IR_OUTPUT_PIN 0
#define SIGNAL "adfasdf"


IRrecv irrecv(IR_INPUT_PIN);
decode_results results;

void setup()
{
  pinMode(IR_OUTPUT_PIN, OUTPUT);
  irrecv.enableIRIn();
}

void loop()
{
  if (irrecv.decode(&results))
  {
    //if(results.value == SIGNAL)
    {
      digitalWrite(IR_OUTPUT_PIN, HIGH);
    }
    irrecv.resume(); // Receive the next value
  }
}