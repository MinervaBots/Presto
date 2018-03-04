#include "Button.h"
#include "Pins.h"
#include "PID.h"
#include "SensorsHelper.h"
#include "MotorControl.h"
#include "Constants.h"
#include "CmdMessenger.h"
#include <EEPROM.h>

struct Configs
{
  float kP, kI, kD;
  unsigned int maxPwm;
  unsigned long timeToStop;
  unsigned long rightLowTime;
  bool halfMotorControl;
};

PID pid;
Configs configs;
Button button(BUTTON_PIN, PULLDOWN);
CmdMessenger cmdMessenger = CmdMessenger(Serial, ',', '\n', '/');

enum State
{
  Idle,
  Calibrating,
  Calibrated,
  Running,
};

bool onLine;
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
  SetRightLowTime = 7,
  Calibrate = 8,
  Start = 9,
  Stop = 10,
  SaveConfigs = 11,
  LoadConfigs = 12,
  SwitchControlMode = 13,
};

void onUnknownCommand(CmdMessenger *messenger);
void onSetKpCommand(CmdMessenger *messenger);
void onSetKiCommand(CmdMessenger *messenger);
void onSetKdCommand(CmdMessenger *messenger);
void onSetMaxPWMCommand(CmdMessenger *messenger);
void onSetMaxTimeCommand(CmdMessenger *messenger);
void onSetRightLowTime(CmdMessenger *messenger);
void onCalibrateCommand(CmdMessenger *messenger);
void onStartCommand(CmdMessenger *messenger);
void onStopCommand(CmdMessenger *messenger);
void onSaveConfigsCommand(CmdMessenger *messenger);
void onLoadConfigsCommand(CmdMessenger *messenger);
void onSwitchControlModeCommand(CmdMessenger *messenger);

void setup()
{
  Serial.begin(BAUD_RATE);
  setupPins();

  pid.setSetPoint(0);
  pid.setSampleTime(5);
  pid.setOutputLimits(-1, 1);
  onLoadConfigsCommand(&cmdMessenger);

  currentState = State::Idle;

  cmdMessenger.attach(onUnknownCommand);
  cmdMessenger.attach(Commands::SetKP, onSetKpCommand);
  cmdMessenger.attach(Commands::SetKI, onSetKiCommand);
  cmdMessenger.attach(Commands::SetKD, onSetKdCommand);
  cmdMessenger.attach(Commands::SetMaxPWM, onSetMaxPWMCommand);
  cmdMessenger.attach(Commands::SetMaxTime, onSetMaxTimeCommand);
  cmdMessenger.attach(Commands::SetRightLowTime, onSetRightLowTime);
  cmdMessenger.attach(Commands::Calibrate, onCalibrateCommand);
  cmdMessenger.attach(Commands::Start, onStartCommand);
  cmdMessenger.attach(Commands::Stop, onStopCommand);
  cmdMessenger.attach(Commands::SaveConfigs, onSaveConfigsCommand);
  cmdMessenger.attach(Commands::LoadConfigs, onLoadConfigsCommand);
  cmdMessenger.attach(Commands::SwitchControlMode, onSwitchControlModeCommand);
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
        resetCalibration();
        currentState = State::Calibrating;
        digitalWrite(LED_PIN, HIGH);

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
        onStartCommand(&cmdMessenger);
      }
      break;

    case State::Running:
      if (readRight())
      {
        // Caso o startTime seja 0, ainda não passamos pela primeira marca
        if (startTime == 0)
        {
          // Então inicia o contador
          startTime = millis();
        }
        // Depois de passar pela primeira marca, só considera a proxima
        // depois de 'rightLowTime' e finaliza o percurso
        else if (millis() - startTime > configs.rightLowTime)
        {
          onStopCommand(&cmdMessenger);
          break;
        }
      }

      if (button.isPressed()/* || (startTime != 0 && (millis() - startTime) > timeToStop)*/)
      {
        onStopCommand(&cmdMessenger);
        break;
      }

      input = readArray(&onLine);
      angularSpeed = -pid.compute(input);

      move(angularSpeed, configs.maxPwm, configs.halfMotorControl);
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
  configs.kP = messenger->readFloatArg();
  configs.kI = pid.getKI();
  configs.kD = pid.getKD();
  pid.setTunings(configs.kP, configs.kI, configs.kD);
  messenger->sendCmd(Commands::Acknowledge, "kP alterado");
}

void onSetKiCommand(CmdMessenger *messenger)
{
  configs.kP = pid.getKP();
  configs.kI = messenger->readFloatArg();
  configs.kD = pid.getKD();
  pid.setTunings(configs.kP, configs.kI, configs.kD);
  messenger->sendCmd(Commands::Acknowledge, "kI alterado");
}

void onSetKdCommand(CmdMessenger *messenger)
{
  configs.kP = pid.getKP();
  configs.kI = pid.getKI();
  configs.kD = messenger->readFloatArg();
  pid.setTunings(configs.kP, configs.kI, configs.kD);
  messenger->sendCmd(Commands::Acknowledge, "kD alterado");
}

void onSetMaxPWMCommand(CmdMessenger *messenger)
{
  configs.maxPwm = messenger->readInt32Arg();
  messenger->sendCmd(Commands::Acknowledge, "Max PWM alterado");
}

void onSetMaxTimeCommand(CmdMessenger *messenger)
{
  configs.timeToStop = messenger->readInt32Arg();
  messenger->sendCmd(Commands::Acknowledge, "Tempo de parada alterado");
}

void onSetRightLowTime(CmdMessenger *messenger)
{
  configs.rightLowTime = messenger->readInt32Arg();
  messenger->sendCmd(Commands::Acknowledge, "Low time alterado");
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
      spin(-1, configs.maxPwm / 2);
    }
    calStart = millis();
    while (millis() - calStart < 1000)
    {
      calibrateSensors();
      spin(1, configs.maxPwm / 2);
    }

    stop();
    delay(300);

    float angularSpeed = -pid.compute(readArray(&onLine));
    while (abs(angularSpeed) > 0.05)
    {
      angularSpeed = -pid.compute(readArray(&onLine));
      move(angularSpeed, configs.maxPwm / 2);
    }

    calStart = millis();
    while (millis() - calStart < 500)
    {
      angularSpeed = -pid.compute(readArray(&onLine));
      move(angularSpeed, configs.maxPwm / 2);
    }
    stop();
    currentState = State::Calibrated;
    cmdMessenger.sendCmd(Commands::Acknowledge, "Fim da calibração");
  }
}

void onStartCommand(CmdMessenger *messenger)
{
  currentState = State::Running;
  cmdMessenger.sendCmd(Commands::Acknowledge, "Inicio do percurso");
  digitalWrite(LED_PIN, HIGH);
  delay(STARTUP_DELAY);
  startTime = 0; //millis();
}

void onStopCommand(CmdMessenger *messenger)
{
  currentState = State::Idle;
  // Gambiarra para que ele pare reto sobre a linha
  move(0, configs.maxPwm);
  delay(50);
  stop();

  unsigned long totalTime = millis() - startTime;

  cmdMessenger.sendCmdStart(Commands::Acknowledge);
  cmdMessenger.sendCmdArg("Fim de percurso - TT");
  cmdMessenger.sendCmdArg(totalTime);
  cmdMessenger.sendCmdEnd();
}

void sendCurrentConfigs(CmdMessenger *messenger)
{
  messenger->sendCmdArg(configs.kP);
  messenger->sendCmdArg(configs.kI);
  messenger->sendCmdArg(configs.kD);
  messenger->sendCmdArg(configs.maxPwm);
  messenger->sendCmdArg(configs.timeToStop);
  messenger->sendCmdArg(configs.rightLowTime);
  messenger->sendCmdArg(configs.halfMotorControl);
}

void onSaveConfigsCommand(CmdMessenger *messenger)
{
  EEPROM.put(0, configs);

  messenger->sendCmdStart(Commands::Acknowledge);
  messenger->sendCmdArg("Configurações salvas - KP, KI, KD, PWM, TTS, RLT, HCM");
  sendCurrentConfigs(messenger);
  messenger->sendCmdEnd();
}

void onLoadConfigsCommand(CmdMessenger *messenger)
{
  EEPROM.get(0, configs);
  pid.setTunings(configs.kP, configs.kI, configs.kD);
  messenger->sendCmdStart(Commands::Acknowledge);
  messenger->sendCmdArg("Configurações carregadas - KP, KI, KD, PWM, TTS, RLT, HCM");
  sendCurrentConfigs(messenger);
  messenger->sendCmdEnd();
}

void onSwitchControlModeCommand(CmdMessenger *messenger)
{
  configs.halfMotorControl = messenger->readBoolArg();
  messenger->sendCmdStart(Commands::Acknowledge);
  messenger->sendCmdArg("Controle do motor alterado");

  if (configs.halfMotorControl)
  {
    pid.setOutputLimits(-0.5, 0.5);
    messenger->sendCmdArg("Half");
  }
  else
  {
    pid.setOutputLimits(-1, 1);
    messenger->sendCmdArg("Full");
  }
  messenger->sendCmdEnd();
}
