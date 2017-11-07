#include <QTRSensors.h>
#include "pins.h"

#define STARTUP_DELAY 300

QTRSensorsRC frontalSensors((unsigned char[]) {2,A5,A4,A3,A2,A1,A0,4}, NUM_SENSORS,1000);
QTRSensorsRC rightSensor((unsigned char[]) {RIGHT_SENSOR_PIN}, 1, 3000);
QTRSensorsRC leftSensor((unsigned char[]) {LEFT_SENSOR_PIN}, 1, 3000);

void setup() {
  Serial.begin(9600);

  setupPins();
  // Calibrar sensores
  while(!button.isPressed());  // Aguarda o botão ser precionado para iniciar calibração

  while(!button.isPressed()) {
    
  }
  calibrateSensors(frontalSensors, rightSensors, leftSensors);

  // Delay inicial
  delay(STARTUP_DELAY);
}

void loop() {
  // Leitura de array de sensores e sensores laterais
  // Cálculo de erro
  // PID (KI = 0)
  // Correção de trajetória
  // Ajustar vel. linear em função do erro
  // Função de parada
  // Outras condições de parada
}

void setupPins(void) {
  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(BUTTON_PIN,INPUT);
  pinMode(R_MOTOR_1,OUTPUT);
  pinMode(R_MOTOR_2,OUTPUT);
  pinMode(L_MOTOR_1,OUTPUT);
  pinMode(L_MOTOR_2,OUTPUT);
  pinMode(LED_PIN,OUTPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  pinMode(A5,INPUT);
  pinMode(2, INPUT):
  pinMode(4, INPUT):
  pinMode(RIGHT_SENSOR_PIN,INPUT);
  pinMode(LEFT_SENSOR_PIN,INPUT);
}

void calibrateSensors(QTRSensorsRC frontal, QTRSensorsRC left, QTRSensorsRC right) {
  frontal.calibrate();
  left.calibrate();
  right.calibrate();
}

