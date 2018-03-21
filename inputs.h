#include <Arduino.h>
#include <Button.h> // needed in the libs folder"
#include <QTRSensors.h> // needed in the libs folder
#include "settings.h"

// Variáveis voláteis para interrupção (contador)
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;

#ifndef INPUTS_H
#define INPUTS_H

Button button(BUTTON);

// ...
#endif

void setupInterrupt()
{
  pinMode(rightEncoderA, INPUT); //OUT A do encoder direito
  pinMode(rightEncoderB, INPUT); //OUT B do encoder direito

  pinMode(leftEncoderA, INPUT);  //OUT A do encoder esquerdo
  pinMode(leftEncoderB, INPUT);  //OUT b do encoder esquerdo

  //=====Interrupções para cada encoder
  // Encoder pin on interrupt 0 (pin 2)  
  // Encoder pin on interrupt 1 (pin 3)
  
  attachInterrupt(0, leftEncoderInterrupt, CHANGE);
  attachInterrupt(1, rightEncoderInterrupt, CHANGE);

  Serial.begin(9600);
}

void debugInterrupt() 
{
  Serial.print("Direita: ");
  Serial.println(rightEncoderCount);
  Serial.print("Esquerda: ");
  Serial.println(leftEncoderCount);
}

void rightEncoderInterrupt()
{
  //Se rightEncoderA e estiver HIGH e rightEncoderB estiver LOW, está indo reto
  
  bool readEncoders = differenceInterrupt(rightEncoderA, rightEncoderB);

  readEncoders ? rightEncoderCount++ : rightEncoder--;
}

void leftEncoderInterrupt()
{
  bool readEncoders = differenceInterrupt(leftEncoderA, leftEncoderB);

  readEncoders ? leftEncoderCount++ : leftEncoder--;
}

bool differenceInterrupt(int encoderA, int encoderB)
{
  return (digitalRead(encoderA) == HIGH && digitalRead(encoderB) == LOW)  //Retorna verdadeiro caso A seja HIGH e B seja LOW
}
