#include "IRremote.h"

#define IR_INPUT_PIN 0
#define IR_OUTPUT_PIN 1
#define KILL_SIGNAL 0x511DBB // Botão channel no controle da robocore


IRrecv irrecv(IR_INPUT_PIN);
decode_results results;

void setup()
{
  pinMode(IR_INPUT_PIN, INPUT);
  pinMode(IR_OUTPUT_PIN, OUTPUT);
  irrecv.enableIRIn();

}

void loop()
{
  if (irrecv.decode(&results))
  {
    if(results.value == KILL_SIGNAL)
    {
      digitalWrite(IR_OUTPUT_PIN, HIGH);
    }
    irrecv.resume(); // Receive the next value
  }
}

