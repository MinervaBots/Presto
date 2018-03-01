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
bool inCurve = false;
int rightMarksCount = 0;

PID pid;
Button button(BUTTON_PIN, PULLDOWN);
CmdMessenger cmdMessenger = CmdMessenger(Serial, ',', '\n', '/');

enum State
{
  Idle,
  Calibrating,
  Calibrated,
  Running,
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

void setup()
{
  Serial.begin(BAUD_RATE);
  setupPins();

  pid.setSetPoint(0);
  pid.setSampleTime(pidSampleTime);
  pid.setOutputLimits(-1, 1);
  //pid.setTunings(kp, ki, kd);
  onLoadConfigsCommand(&cmdMessenger);

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
}

void loop()
{
  float input, angularSpeed;
  cmdMessenger.feedinSerialData();

  switch (currentState)
  {
    case State::Idle:
      if (button.isPressed())
      {
        currentState = State::Calibrating;
        digitalWrite(LED_PIN, HIGH);
        resetCalibration();
        cmdMessenger.sendCmd(Commands::Acknowledge, "Iniciando calibração");
        delay(DEBOUNCE_TIME);
      }
      break;

    case State::Calibrating:
      calibrateSensors();
      if (button.isPressed())
      {
        currentState = State::Calibrated;
        cmdMessenger.sendCmd(Commands::Acknowledge, "Fim da calibração");
        digitalWrite(LED_PIN, LOW);
        delay(DEBOUNCE_TIME);
      }
      break;

    case State::Calibrated:
      if (button.isPressed())
      {
        currentState = State::Running;
        digitalWrite(LED_PIN, HIGH);
        delay(STARTUP_DELAY);
        startTime = millis();
      }
      break;

    case State::Running:
      bool leftMark;
      bool rightMark;

      readLaterals(&leftMark, &rightMark);
      if (rightMark && leftMark)
      {
        // Interseção. Não faz nada
      }
      else if (rightMark && !leftMark)
      {
        // Incrementa as marcas de parada
        rightMarksCount++;
      }
      else if (!rightMark && leftMark)
      {
        // Verifica a curva
        inCurve = !inCurve;
        if (inCurve)
        {
          digitalWrite(LED_PIN, HIGH);
          pid.setTunings(kp * 1.5, ki, kd);
        }
        else
        {
          digitalWrite(LED_PIN, LOW);
          pid.setTunings(kp, ki, kd);
        }
      }

      if (button.isPressed() || rightMarksCount >= rightMarksToStop || (millis() - startTime) > timeToStop)
      {
        currentState = State::Idle;
        // Tira qualquer velocidade angular para que ele pare reto
        move(0, maxPwm);
        delay(50);
        stop();

        unsigned long totalTime = millis() - startTime;

        cmdMessenger.sendCmdStart(Commands::Acknowledge);
        cmdMessenger.sendCmdArg("Fim de percurso - TT, MD");
        cmdMessenger.sendCmdArg(totalTime);
        cmdMessenger.sendCmdArg(rightMarksCount);
        cmdMessenger.sendCmdEnd();

        rightMarksCount = 0;
        break;
      }
      
      input = readArray();
      angularSpeed = -pid.compute(input);

      move(angularSpeed, maxPwm);
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
  kp = messenger->readFloatArg();
  ki = pid.getKI();
  kd = pid.getKD();
  pid.setTunings(kp, ki, kd);
  messenger->sendCmd(Commands::Acknowledge, "kP alterado");
}

void onSetKiCommand(CmdMessenger *messenger)
{
  kp = pid.getKP();
  ki = messenger->readFloatArg();
  kd = pid.getKD();
  pid.setTunings(kp, ki, kd);
  messenger->sendCmd(Commands::Acknowledge, "kI alterado");
}

void onSetKdCommand(CmdMessenger *messenger)
{
  kp = pid.getKP();
  ki = pid.getKI();
  kd = messenger->readFloatArg();
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
    resetCalibration();
    cmdMessenger.sendCmd(Commands::Acknowledge, "Iniciando calibração");
    unsigned long calStart = millis();
    while (millis() - calStart < 1000)
    {
      calibrateSensors();
      spin(-1, maxPwm / 2);
    }
    calStart = millis();
    while (millis() - calStart < 1000)
    {
      calibrateSensors();
      spin(1, maxPwm / 2);
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
    while (millis() - calStart < 500)
    {
      angularSpeed = -pid.compute(readArray());
      move(angularSpeed, maxPwm / 2);
    }
    stop();
    currentState = State::Calibrated;
    cmdMessenger.sendCmd(Commands::Acknowledge, "Fim da calibração");
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
  stop();
  messenger->sendCmd(Commands::Acknowledge, "Finalizado");
}

void sendCurrentConfigs(CmdMessenger *messenger)
{
  messenger->sendCmdArg(kp);
  messenger->sendCmdArg(ki);
  messenger->sendCmdArg(kd);
  messenger->sendCmdArg(maxPwm);
  messenger->sendCmdArg(timeToStop);
  messenger->sendCmdArg(rightMarksToStop);
}

void onSaveConfigsCommand(CmdMessenger *messenger)
{
  EEPROM.put(0 * sizeof(float), kp);
  EEPROM.put(1 * sizeof(float), ki);
  EEPROM.put(2 * sizeof(float), kd);
  EEPROM.put(3 * sizeof(float), maxPwm);
  EEPROM.put(3 * sizeof(float) + 1 * sizeof(int), timeToStop);
  EEPROM.put(3 * sizeof(float) + 2 * sizeof(int), rightMarksToStop);

  messenger->sendCmdStart(Commands::Acknowledge);
  messenger->sendCmdArg("Configurações salvas - KP, KI, KD, PWM, TTS, MTS");
  sendCurrentConfigs(messenger);
  messenger->sendCmdEnd();
}

void onLoadConfigsCommand(CmdMessenger *messenger)
{
  EEPROM.get(0 * sizeof(float), kp);
  EEPROM.get(1 * sizeof(float), ki);
  EEPROM.get(2 * sizeof(float), kd);
  EEPROM.get(3 * sizeof(float), maxPwm);
  EEPROM.get(3 * sizeof(float) + 1 * sizeof(int), timeToStop);
  EEPROM.get(3 * sizeof(float) + 2 * sizeof(int), rightMarksToStop);

  pid.setTunings(kp, ki, kd);
  messenger->sendCmdStart(Commands::Acknowledge);
  messenger->sendCmdArg("Configurações carregadas - KP, KI, KD, PWM, TTS, MTS");
  sendCurrentConfigs(messenger);
  messenger->sendCmdEnd();
}
