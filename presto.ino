//falta programar o inicio (parte da calibração
#include <Button.h>

#include "pins.h"
#include "PID.h"
#include "SensorRead.h"

#define STARTUP_DELAY 300

unsigned int sensor_values[8] = {0,0,0,0,0,0,0,0};

Button button(BUTTON_PIN, BUTTON_PULLDOWN);

//não tenho certeza de onde colocar essas variáveis 
int start = 1;
int leitura = 0;
float stopTime = 0;

void setupPins(void);

void setup() {
  Serial.begin(9600);

  setupPins();
  // Calibrar sensores
  while(!button.isPressed());  // Aguarda o botão ser precionado para iniciar calibração

  Serial.println("Botão Apertado");

  // Tempo de debounce
  delay(500);
  
  digitalWrite(LED_PIN, HIGH);

  Serial.println("Começando a calibrar");
  while(!button.isPressed()) {
    Serial.println("Calibrando");
    calibrateSensors(&frontalSensors, &rightSensor, &leftSensor);
  }
  Serial.println("Fim da calibração");
  Serial.println(LED_PIN, LOW);

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
  if(start){
    
    //leitura do bluetooth
    if(Serial.available()> 0){
      leitura = Serial.read();
    }

    frontalSensors.read(Sensors);
    int error = calculateError(NUM_SENSORS, Sensors);
    Serial.print("Erro: ");
    Serial.println(error);
    delay(500);
    angularSpeed = controllerPID(error);
    Serial.print("angularSpeed: ");
    Serial.println(angularSpeed);
    
    motorController(1,angularSpeed);

    //função de leitura do máximo da direita
    if(rightCount == MAXCOUNT){
      start = 0;
      stopTime= millis();
    }
    
    // função de parada pelo bluetooth
    if(leitura != 49){
      start = 0;
      stopTime = millis();
    }
    
  }
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
  pinMode(2,INPUT);
  pinMode(4,INPUT);
  pinMode(RIGHT_SENSOR_PIN,INPUT);
  pinMode(LEFT_SENSOR_PIN,INPUT);
}



