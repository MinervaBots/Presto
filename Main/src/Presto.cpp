#include "Includes.h"

const float kp = 165;
const float ki = 0;
const float kd = 0.25;
const int maxVelocity = 150;
const int stopTime = 10000;
const int rightBorderMarksLimit = 100;
const LineColor lineColor = LineColor::White;


volatile bool killSwitchSignal = false;
volatile bool encoderForwardTicksCount = 0;
volatile bool encoderBackwardTicksCount = 0;


LineFollower presto;
PrestoSensoring sensoring;
Button commandButton(COMMAND_BUTTON_PIN, PULLDOWN);
PIDController pidController;
PrestoMotorController motorController;


void encoderInterruption();
void killswitchInterruption();


void setup()
{
  Serial.begin(9600);

  pinMode(KILLSWITCH_PIN, INPUT_PULLUP);
  enableInterrupt(KILLSWITCH_PIN, &killswitchInterruption, RISING);
  enableInterrupt(ENCODER_PIN, &encoderInterruption, CHANGE);

  presto.setStatusPin(STATUS_LED_PIN);

  pidController.setSetPoint(0);
  pidController.setSampleTime(5);
  pidController.setOutputLimits(-255, 255);
  pidController.setControllerDirection(SystemControllerDirection::Direct);
  pidController.setTunings(kp, ki, kd); //CONSTANTES PID 290, 0, 270
  presto.setSystemController(&pidController);

  motorController.setPins(LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2);

  presto.setMotorController(&motorController);

  sensoring.setLineColor(lineColor);
  sensoring.setSensorArray(SensorArrayPins, arraySize(SensorArrayPins), 500);
  sensoring.setLeftSensor(LeftBorderSensorPin, 120, 3000);
  sensoring.setRightSensor(RightBorderSensorPin, 150, 3000);

  presto.setInputSource(&sensoring);

  sensoring.calibrate(commandButton, STATUS_LED_PIN);
  presto.setLinearVelocity(maxVelocity);
  presto.start();
}


void loop()
{
  /*
  Passando em 5 marcas do lado direito ou 20 segundos de prova ou o sinal
  do killswitch paramos o Presto.
  */
  if(/*sensoring.shouldStop(rightBorderMarksLimit) || */presto.shouldStop(stopTime)/* || killSwitchSignal*/)
  {
    if(presto.getIsRunning())
    {
      presto.stop();
      Serial.println("Parou");
    }
    else
    {
      // Aguarda apertar de novo pra recomeçar a execução
      while(!commandButton.isPressed());

      //delay(250); // Sem essa linha, muito provavelmente ele vai pular direto pro while de calibração
      sensoring.calibrate(commandButton, STATUS_LED_PIN);
      killSwitchSignal = false;
      presto.start();
    }
    return;
  }

  sensoring.update();
  /*
  Se estivermos em curva reduz a velocidade linear pela metade
  */
  //presto.setLinearVelocity(sensoring.inCurve() ? 75 : 150);
  presto.update();
}


//--Interruptions---------------------------------------------------------------

void killswitchInterruption()
{
  Serial.println("Killswitch");
  killSwitchSignal = true;
}


void encoderInterruption()
{
  if(motorController.IsLeftForward())
    encoderForwardTicksCount++;
  else
    encoderBackwardTicksCount++;
}

//-----------------------------------------------------------------------------
