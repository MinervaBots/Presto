#include <Arduino.h>
#include <Button.h> // needed in the libs folder"
#include <QTRSensors.h> // needed in the libs folder
#include "settings.h"

// Variáveis voláteis para interrupção 
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;
volatile bool leftInterruptOccured = false;
volatile bool rightInterruptOccured = false;

unsigned long curveTime[CURVES_NUMBER];
unsigned long lastRightInterruptOccured;
unsigned long lastLeftInterruptOccured;
bool curve;
int listNumber = 0;

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

//===== Início das funções de interrupção
void rightEncoderInterrupt()
{
  //Se rightEncoderA e estiver HIGH e rightEncoderB estiver LOW, está indo reto
  
  bool readEncoders = differenceInterrupt(rightEncoderA, rightEncoderB);
  leftInterruptOccurred = true;

  readEncoders ? rightEncoderCount++ : rightEncoder--;
}

void leftEncoderInterrupt()
{
  bool readEncoders = differenceInterrupt(leftEncoderA, leftEncoderB);
  rightInterruptOccurred = true;

  readEncoders ? leftEncoderCount++ : leftEncoder--;
}
//===== Fim das funções de Interrupção


bool differenceInterrupt(int encoderA, int encoderB)
{
  return (digitalRead(encoderA) == HIGH && digitalRead(encoderB) == LOW)  //Retorna verdadeiro caso A seja HIGH e B seja LOW
}

void lineRoutine()                                                                                          // Função para saber se está numa curva ou reta
{
  if (interruptOccurred == true)                                                                            // Caso ele tenha feito uma leitura do encoder
  {
    if((rightEncoderCount - lastRightEncoderCount) == 1 && (leftEncoderCount - lastLeftEncoderCount) == 1)  // Caso ele esteja vendo a mesma quantidade de variação nos dois encoders
    {
      if(curve = true)                                                                                      // Caso ele tenha saído de curva
      {
        curveTime[listNumber];                                                                              // Adiciona na lista o início da reta(fim da curva)
        listNumber++;
      }
      curve = false;                                                                                        // Seta o início da reta
    
    }
    if not ((rightEncoderCount - lastRightEncoderCount) == 1 && (leftEncoderCount - lastLeftEncoderCount) == 1 && curve == false)  // Se ele não estiver numa curva, e tiver acabado de sair de uma reta
    {
      curveTime[listNumber] = millis();                                                                     // Adiciona o tempo de início da curva
      listNumber++;
      curve = true;                                                                                         // Seta o início da curva
    }
    interruptOccurred = false;
    lastRightEncoderCount = RightEncoderCount;
    lastleftEncoderCount = leftEncoderCount;
  }
}
