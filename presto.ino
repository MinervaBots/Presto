#include "Button.h"
#include "Pins.h"
#include "PID.h"
#include "SensorsHelper.h"
#include "MotorControl.h"

#include "Constants.h"
#include "CmdMessenger.h"
#include <EEPROM.h>

bool reta = true;

int tamanho = 2;//18;
bool pista[] = {0,0};//{0,1,0,1,1,0,0,1,0,1,0,0,1,0,1,0,1,0};
int estado = 0;

struct Configs
{
  float kP, kI, kD;
  unsigned int maxPwm;
  unsigned int curvePwm;
  unsigned long timeToStop;
  unsigned long rightLowTime;
  bool halfMotorControl;
};

PID pid;
Configs configs;
Button button(BUTTON_PIN);//, PULLDOWN);
CmdMessenger cmdMessenger = CmdMessenger(Serial3, ',', '\n', '/');

enum State
{
  Idle,
  Calibrating,
  Calibrated,
  Running,
  TuningPID,
};

State currentState;
unsigned long startTime, stopTime;


// Vars para o algoritmo Twiddle de auto tune do PID
// Colocar alguns parâmetros que sabemos que funcionam razoavelmente bem
// Em ordem: P, I ,D
float parameters[3] = {0.7, 0, 0};
float dParameters[3] = {0.1, 0.005, 1};
float dPSum = (dParameters[0] + dParameters[1] + dParameters[2]);
// Quando a variação total dos parâmetros for menor do que esse limitante o código para
float dPThreshold = 0.000001f;
float error = 0;
float bestError = 0;
unsigned long lastDataSentTime = 0;
void sendAutoTuneState();

void setupPins();
void calibrate();
void waitButtonPress();
float followPath(int maxPwm, int curvePwm);

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
  AutoTunePID = 14,
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
void onAutoTunePIDCommand(CmdMessenger *messenger);

void setup()
{
  //Serial.begin(9600);
  Serial3.begin(BAUD_RATE);
  setupPins();
  pid.setSetPoint(0);
  pid.setSampleTime(1);
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
  cmdMessenger.attach(Commands::AutoTunePID, onAutoTunePIDCommand);
  //configs.rightLowTime = 1000*(15 + 3*(digitalReadFast(9) + digitalReadFast(10) + digitalReadFast(11) + digitalReadFast(12)));


}
void loop()
{
  //Serial.println(readRight());
//  float input, angularSpeed;
  cmdMessenger.feedinSerialData();

  switch (currentState)
  {
    case State::Idle:
      if (button.isPressed())
      {
        resetCalibration();
        currentState = State::Calibrating;
        digitalWriteFast(LED_PIN, HIGH);

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
        digitalWriteFast(LED_PIN, LOW);
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
      if (startTime == 0)
        {
          startTime = millis();
        }
      if (millis() - startTime > configs.timeToStop) {
        onStopCommand(&cmdMessenger);
        break;
      }
      if (readRight())
      {
        //digitalWriteFast(LED_PIN, HIGH);
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
      else {
        //digitalWriteFast(LED_PIN, LOW);
        //digitalWriteFast(24, LOW);
      }
      
      if (button.isPressed())
      {
        onStopCommand(&cmdMessenger);
        break;
      }
      followPath(configs.maxPwm);
      break;
      
    case State::TuningPID:
      for (int i = 0; i < 3; i++)
      {
        parameters[i] += dParameters[i];
        pid.setTunings(parameters);
        
        error = followPath(configs.maxPwm);
  
        if (error < bestError)
        {
          bestError = error;
          dParameters[i] *= 1.1;
        }
        else
        {
          parameters[i] -= 2 * dParameters[i];
          pid.setTunings(parameters);
          
          error = followPath(configs.maxPwm);
  
          if (error < bestError)
          {
            bestError = error;
            dParameters[i] *= 1.05f;
          }
          else
          {
            parameters[i] += dParameters[i];
            dParameters[i] *= 0.95;
          }
        }
        
      }
      dPSum = (dParameters[0] + dParameters[1] + dParameters[2]);

      if(millis() - lastDataSentTime > 3000)
      {
        lastDataSentTime = millis();
        sendAutoTuneState();
      }

      if (button.isPressed() || dPSum > dPThreshold)
      {
        currentState = State::Idle;
        configs.kP = parameters[0];
        configs.kI = parameters[1];
        configs.kD = parameters[2];
        sendAutoTuneState();
        EEPROM.put(0, configs);
      }
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
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BATTERY, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11,INPUT);
  pinMode(12, INPUT);
  pinMode(25, OUTPUT);
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
  //configs.curvePwm = messenger->readInt32Arg();
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

    float angularSpeed = -pid.compute(readArray());
    while (abs(angularSpeed) > 0.05)
    {
      angularSpeed = -pid.compute(readArray());
      move(angularSpeed, configs.maxPwm / 2);
    }

    calStart = millis();
    while (millis() - calStart < 500)
    {
      angularSpeed = -pid.compute(readArray());
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
  digitalWriteFast(LED_PIN, HIGH);
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
  estado = 0;

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
  //messenger->sendCmdArg(configs.curvePwm);
  messenger->sendCmdArg(configs.timeToStop);
  messenger->sendCmdArg(configs.rightLowTime);

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

void onAutoTunePIDCommand(CmdMessenger *messenger)
{
  currentState = State::TuningPID;
  bestError = followPath(configs.maxPwm);
}

float followPath(int maxPwm)//, int curvePwm)
{
  static bool ledState = true;
  //move(angularSpeed, maxPwm + (ledState - 1)*50, configs.halfMotorControl);
  float input = readArray();
  //float angularSpeed = -pid.compute(input); //invertido ver SensorHelper
  if (changeState()) {
    //ledState=not(ledState);
    if (estado < tamanho) {
      estado++;
    }
    else {
      //onStopCommand(&cmdMessenger);
      estado = 1;
    }
    /*
    if (pista[estado]) {
      analogWrite(L_MOTOR_1, 0);
      analogWrite(L_MOTOR_2, 255);
      analogWrite(R_MOTOR_1, 255);
      analogWrite(R_MOTOR_2, 0);
      delay(50);
    }
    */
    
  }
  if (pista[estado]) {//(ledState==false){
    pid.setTunings(KP_CURVA, 0, KD_CURVA);
    float angularSpeed = pid.compute(input);
    move(angularSpeed, PWM_CURVA, configs.halfMotorControl);//curvePwm, configs.halfMotorControl);
  }
  
  else{
   // configs.kP = KP_RETA;
   // configs.kI = KI_RETA;
   // configs.kD = KD_RETA;
    pid.setTunings(configs.kP, configs.kI, configs.kD);
    float angularSpeed = pid.compute(input);
    move(angularSpeed, maxPwm, configs.halfMotorControl);
  }
  return abs(input);

}

void sendAutoTuneState()
{
  cmdMessenger.sendCmdStart(Commands::Acknowledge);
  cmdMessenger.sendCmdArg(parameters[0]);
  cmdMessenger.sendCmdArg(parameters[1]);
  cmdMessenger.sendCmdArg(parameters[2]);
  
  cmdMessenger.sendCmdArg(dParameters[0]);
  cmdMessenger.sendCmdArg(dParameters[1]);
  cmdMessenger.sendCmdArg(dParameters[2]);
  
  cmdMessenger.sendCmdArg(dPSum);
  cmdMessenger.sendCmdEnd();
}
