#include "Button.h"
#include "Pins.h"
#include "PID.h"
#include "SensorsHelper.h"
#include "MotorControl.h"
#include "Constants.h"
#include "CmdMessenger.h"
#include <EEPROM.h>

float kp = 0.35;
float ki = 0;
float kd = 15000;
int maxPwm = 100;
unsigned long rightMarksToStop = 4;
unsigned long timeToStop = 20000; // 20s

const unsigned long pidSampleTime = 5;

PID pid;
Button button(BUTTON_PIN, PULLDOWN);
CmdMessenger cmdMessenger = CmdMessenger(Serial);

enum State
{
  Idle,
  Calibrating,
  Calibrated,
  Running,
  ExperimentalTuning
};

State currentState;
unsigned long startTime, stopTime;

void setupPins();
void calibrate();
void waitButtonPress();

enum Commands
{
  Acknowledge = 0,
  Error = 1,
  SetKP = 2,
  SetKI = 3,
  SetKD = 4,
  SetMaxPWM = 5,
  SetMaxTime = 6,
  SetMaxRightMarks = 7,
  Calibrate = 8,
  Start = 9,
  Stop = 10,
  SaveConfigs = 11,
  LoadConfigs = 12,
  StartTuning = 13
};

void onUnknownCommand(CmdMessenger *messenger);
void onSetKpCommand(CmdMessenger *messenger);
void onSetKiCommand(CmdMessenger *messenger);
void onSetKdCommand(CmdMessenger *messenger);
void onSetMaxPWMCommand(CmdMessenger *messenger);
void onSetMaxTimeCommand(CmdMessenger *messenger);
void onSetMaxRightMarksCommand(CmdMessenger *messenger);
void onCalibrateCommand(CmdMessenger *messenger);
void onStartCommand(CmdMessenger *messenger);
void onStopCommand(CmdMessenger *messenger);
void onSaveConfigsCommand(CmdMessenger *messenger);
void onLoadConfigsCommand(CmdMessenger *messenger);
void onStartTuningCommand(CmdMessenger *messenger);


void setup()
{
  /*
    // Muda o pre-scale do clock do ADC
    // Em teoria descomentar isso faria a leitura
    // dos sensores laterais ser 8x mais rápida
    sbi(ADCSRA, ADPS2);
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0);
  */

  Serial.begin(BAUD_RATE);
  setupPins();

  pid.setSetPoint(0);
  pid.setSampleTime(pidSampleTime);
  pid.setOutputLimits(-1, 1);
  pid.setTunings(kp, ki, kd);

  currentState = State::Idle;

  cmdMessenger.attach(onUnknownCommand);
  cmdMessenger.attach(Commands::SetKP, onSetKpCommand);
  cmdMessenger.attach(Commands::SetKI, onSetKiCommand);
  cmdMessenger.attach(Commands::SetKD, onSetKdCommand);
  cmdMessenger.attach(Commands::SetMaxPWM, onSetMaxPWMCommand);
  cmdMessenger.attach(Commands::SetMaxTime, onSetMaxTimeCommand);
  cmdMessenger.attach(Commands::SetMaxRightMarks, onSetMaxRightMarksCommand);
  cmdMessenger.attach(Commands::Calibrate, onCalibrateCommand);
  cmdMessenger.attach(Commands::Start, onStartCommand);
  cmdMessenger.attach(Commands::Stop, onStopCommand);
  cmdMessenger.attach(Commands::SaveConfigs, onSaveConfigsCommand);
  cmdMessenger.attach(Commands::LoadConfigs, onLoadConfigsCommand);
  cmdMessenger.attach(Commands::StartTuning, onStartTuningCommand);
}

void loop()
{
  float input, angularSpeed;
  cmdMessenger.feedinSerialData();
  //cmdMessenger.sendCmd(Commands::Acknowledge, "ola");
  switch (currentState)
  {
    case State::Idle:
      if (button.isPressed())
      {
        currentState = State::Calibrating;
        digitalWrite(LED_PIN, HIGH);
        Serial.println("Iniciando calibração");
        delay(DEBOUNCE_TIME);
      }
      break;

    case State::Calibrating:
      calibrateSensors();
      if (button.isPressed())
      {
        currentState = State::Calibrated;
        Serial.println("Fim da calibração");
        digitalWrite(LED_PIN, LOW);
        delay(DEBOUNCE_TIME);
      }
      break;

    case State::Calibrated:
      if (button.isPressed())
      {
        currentState = State::Running;
        Serial.println("Fim da calibração");
        digitalWrite(LED_PIN, HIGH);
        delay(STARTUP_DELAY);
      }
      break;

    case State::Running:
      if (button.isPressed() || readRight(rightMarksToStop) || (millis() - startTime) > timeToStop)
      {
        currentState = State::Idle;
        // Tira qualquer velocidade angular para que ele pare reto
        move(0, maxPwm);
        delay(50);
        stop();

        stopTime = millis();
        Serial.print("Tempo total: ");
        Serial.println(stopTime - startTime);
        resetRightCount();
        break;
      }

      input = readArray();
      angularSpeed = -pid.compute(input);

      /*
        Serial.print("Input: ");
        Serial.println(input);
        Serial.print("PID: ");
        Serial.println(angularSpeed);
      */

      move(angularSpeed, maxPwm);
      break;
      
   case State::ExperimentalTuning:
      // Tentar auto tune de PID?
      break;
  }
}

void setupPins()
{
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(R_MOTOR_1, OUTPUT);
  pinMode(R_MOTOR_2, OUTPUT);
  pinMode(L_MOTOR_1, OUTPUT);
  pinMode(L_MOTOR_2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
}

void onUnknownCommand(CmdMessenger *messenger)
{
  messenger->sendCmd(Commands::Error, "Command without attached callback");
}

void onSetKpCommand(CmdMessenger *messenger)
{
  float kp = messenger->readFloatArg();
  float ki = pid.getKI();
  float kd = pid.getKD();
  pid.setTunings(kp, ki, kd);
  messenger->sendCmd(Commands::Acknowledge, "kP alterado");
}

void onSetKiCommand(CmdMessenger *messenger)
{
  float kp = pid.getKP();
  float ki = messenger->readFloatArg();
  float kd = pid.getKD();
  pid.setTunings(kp, ki, kd);
  messenger->sendCmd(Commands::Acknowledge, "kI alterado");
}

void onSetKdCommand(CmdMessenger *messenger)
{
  float kp = pid.getKP();
  float ki = pid.getKI();
  float kd = messenger->readFloatArg();
  pid.setTunings(kp, ki, kd);
  messenger->sendCmd(Commands::Acknowledge, "kD alterado");
}

void onSetMaxPWMCommand(CmdMessenger *messenger)
{
  maxPwm = messenger->readInt32Arg();
  messenger->sendCmd(Commands::Acknowledge, "Max PWM alterado");
}

void onSetMaxTimeCommand(CmdMessenger *messenger)
{
  timeToStop = messenger->readInt32Arg();
  messenger->sendCmd(Commands::Acknowledge, "Tempo de parada alterado");
}

void onSetMaxRightMarksCommand(CmdMessenger *messenger)
{
  rightMarksToStop = messenger->readInt32Arg();
  messenger->sendCmd(Commands::Acknowledge, "Numero de marcações alterado");
}


void onCalibrateCommand(CmdMessenger *messenger)
{
  if (currentState == State::Idle)
  {
    unsigned long calStart = millis();
    while (millis() - calStart < 3000)
    {
      calibrateSensors();
      spin(-1, maxPwm / 2.5);
    }
    calStart = millis();
    while (millis() - calStart < 3000)
    {
      calibrateSensors();
      spin(1, maxPwm / 2.5);
    }

    stop();
    delay(300);

    float angularSpeed = -pid.compute(readArray());
    while (abs(angularSpeed) > 0.05)
    {
      angularSpeed = -pid.compute(readArray());
      move(angularSpeed, maxPwm / 2);
    }
    
    calStart = millis();
    while (millis() - calStart < 1000)
    {
      angularSpeed = -pid.compute(readArray());
      move(angularSpeed, maxPwm / 2);
    }
    stop();
    currentState = State::Calibrated;
  }
}

void onStartCommand(CmdMessenger *messenger)
{
  currentState = State::Running;
  startTime = millis();
  messenger->sendCmd(Commands::Acknowledge, "Iniciado");
}

void onStopCommand(CmdMessenger *messenger)
{
  currentState = State::Idle;
  messenger->sendCmd(Commands::Acknowledge, "Finalizado");
}

void onSaveConfigsCommand(CmdMessenger *messenger)
{
  EEPROM.put(0 * sizeof(float), pid.getKP());
  EEPROM.put(2 * sizeof(float), pid.getKI());
  EEPROM.put(1 * sizeof(float), pid.getKD());
  EEPROM.put(3 * sizeof(float), maxPwm);
  EEPROM.put(3 * sizeof(float) + 1 * sizeof(int), timeToStop);
  EEPROM.put(3 * sizeof(float) + 2 * sizeof(int), rightMarksToStop);
  
  messenger->sendCmdStart(Commands::Acknowledge);
  messenger->sendCmdArg("Configurações salvas - KP, KI, KD, PWM, TTS, MTS");
  messenger->sendCmdArg(pid.getKP());
  messenger->sendCmdArg(pid.getKI());
  messenger->sendCmdArg(pid.getKD());
  messenger->sendCmdArg(maxPwm);
  messenger->sendCmdArg(timeToStop);
  messenger->sendCmdArg(rightMarksToStop);
  messenger->sendCmdEnd();
}

void onLoadConfigsCommand(CmdMessenger *messenger)
{
  float kp;
  float ki;
  float kd;
  EEPROM.get(0 * sizeof(float), kp);
  EEPROM.get(1 * sizeof(float), ki);
  EEPROM.get(2 * sizeof(float), kd);
  EEPROM.get(3 * sizeof(float), maxPwm);
  EEPROM.get(3 * sizeof(float) + 1 * sizeof(int), timeToStop);
  EEPROM.get(3 * sizeof(float) + 2 * sizeof(int), rightMarksToStop);

  pid.setTunings(kp, ki, kd);
  messenger->sendCmdStart(Commands::Acknowledge);
  messenger->sendCmdArg("Configurações carregadas - KP, KI, KD, PWM, TTS, MTS");
  messenger->sendCmdArg(kp);
  messenger->sendCmdArg(ki);
  messenger->sendCmdArg(kd);
  messenger->sendCmdArg(maxPwm);
  messenger->sendCmdArg(timeToStop);
  messenger->sendCmdArg(rightMarksToStop);
  messenger->sendCmdEnd();
}

void onStartTuningCommand(CmdMessenger *messenger)
{
  currentState = State::ExperimentalTuning;
}

