#include "IRremote.h"

#define IR_INPUT_PIN A2
#define IR_OUTPUT_PIN 0
#define HIGH_PIN 3
#define KILL_SIGNAL 0x511DBB // Botão channel no controle da robocore

IRrecv irrecv(IR_INPUT_PIN);
decode_results results;

void setup()
{
  pinMode(IR_INPUT_PIN, INPUT);
  pinMode(IR_OUTPUT_PIN, OUTPUT);
  pinMode(HIGH_PIN, OUTPUT);
  digitalWrite(HIGH_PIN, HIGH);
  irrecv.enableIRIn();
  Serial.begin(9600);
}

void loop()
{
  if (irrecv.decode(&results))
  {
    Serial.println(results.value);
    if(results.value == KILL_SIGNAL)
    {
      digitalWrite(IR_OUTPUT_PIN, HIGH);
      delay(200);
      digitalWrite(IR_OUTPUT_PIN, LOW);
    }
    irrecv.resume(); // Receive the next value
  }
}

