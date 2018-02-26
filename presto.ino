
//falta programar o inicio (parte da calibração
#include "Button.h"
#include "pins.h"
#include "PID.h"
#include "SensorRead.h"
#include "MotorFunction.h"

#define STARTUP_DELAY 300
#define DEBOUNCE_TIME 500

unsigned int sensor_values[8] = {0, 0, 0, 0, 0, 0, 0, 0};

Button button(BUTTON_PIN, PULLDOWN);

//não tenho certeza de onde colocar essas variáveis
int start = 1;
int leitura = 0;
float stopTime = 0;
float angularSpeedCalibrate = 0;

void setupPins(void);

void setup()
{
  Serial.begin(9600);
  setupPins();

  // Calibrar sensores
  while (!button.isPressed()); // Aguarda o botão ser pressionado para iniciar calibração

  Serial.println("Botão Apertado");
  digitalWrite(LED_PIN, HIGH);
  delay(DEBOUNCE_TIME);


  Serial.println("Iniciando calibração");
  while (!button.isPressed())
  {
    Serial.println("Calibrando...");
    calibrateSensors(&frontalSensors, &rightSensor, &leftSensor);
  }
  digitalWrite(LED_PIN, LOW);
  Serial.println("Fim da calibração");
  delay(DEBOUNCE_TIME);
  
  // Aguarda botão para iniciar
  while (!button.isPressed());
  digitalWrite(LED_PIN, HIGH);
  delay(STARTUP_DELAY);
}

void loop()
{
  // Leitura de array de sensores e sensores laterais
  // Cálculo de erro
  // PID (KI = 0)
  // Correção de trajetória
  // Ajustar vel. linear em função do erro
  // Função de parada
  // Outras condições de parada
  while (start)
  {
    readRight();
    if(rightCount >= 4)
    {
      start = false;
      motorController(0);
      delay(50);
      break;
    }
    
    int line = frontalSensors.readLine(Sensors);
    /*
    if(line < 0)
    {
      
    }
    else if(line > 7000)
    {
      
    }
    */
    float input = (line - centerPosition) / 1000;
    float angularSpeed = -controllerPID(input);
    /*
    Serial.print("Input: ");
    Serial.println(input);
    Serial.print("PID: ");
    Serial.println(angularSpeed);
    */
    motorController(angularSpeed);
    // função de leitura do máximo da direita

    // função de parada pelo bluetooth
  }
  stop();
}

void setupPins()
{
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(R_MOTOR_1, OUTPUT);
  pinMode(R_MOTOR_2, OUTPUT);
  pinMode(L_MOTOR_1, OUTPUT);
  pinMode(L_MOTOR_2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
}



