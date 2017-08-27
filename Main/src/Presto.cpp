#include "Includes.h"
/*
primeiro dia tomada de tempo-testes
const float kp = 202;
const float ki = 0.00;
const float kd = 1940;

const int maxVelocity = 125;
const int stopTime = 20000;
*/

/*
testes do dia da inspeção-testes
*/

const int maxVelocity = 115;
const int stopTime = 20000;
const float kp = 205;
const float ki = 0;
const float kd = 2400;


const int rightSensorTime = 20000;
const int rightBorderMarksLimit = 500;
const LineColor lineColor = LineColor::White;


volatile bool killSwitchSignal = false;
volatile bool encoderForwardTicksCount = 0;
volatile bool encoderBackwardTicksCount = 0;


LineFollower presto;
PrestoSensoring sensoring(&Serial);     // Serial is used by PrestoSensoring to interface with the bluetooth module
Button commandButton(COMMAND_BUTTON_PIN, PULLDOWN);
PIDController pidController;
PrestoMotorController motorController;


void encoderInterruption();
void killswitchInterruption();


void setup()
{
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(KILLSWITCH_PIN, INPUT_PULLUP);
  //enableInterrupt(KILLSWITCH_PIN, &killswitchInterruption, RISING);
  //enableInterrupt(ENCODER_PIN, &encoderInterruption, CHANGE);
  presto.setStatusPin(STATUS_LED_PIN);

  pidController.setSetPoint(0);
  pidController.setSampleTime(5);
  pidController.setOutputLimits(-255 - maxVelocity, 255 + maxVelocity);
  pidController.setControllerDirection(SystemControllerDirection::Direct);
  pidController.setTunings(kp, ki, kd); //CONSTANTES PID 290, 0, 270
  presto.setSystemController(&pidController);

  motorController.setPins(LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2);

  presto.setMotorController(&motorController);

  sensoring.setLineColor(lineColor);
  sensoring.setSensorArray(SensorArrayPins, arraySize(SensorArrayPins), 200);
  sensoring.setLeftSensor(LeftBorderSensorPin, 0, 3000);
  sensoring.setRightSensor(RightBorderSensorPin, rightSensorTime, 1000);

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
  if(sensoring.shouldStop(rightBorderMarksLimit) || sensoring.killswitch() || presto.shouldStop(stopTime))
  {
    if(presto.getIsRunning())
    {
      presto.stop();
      Serial.println("Parou");
      delay(1000);
      digitalWrite(BUZZER_PIN, LOW);
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
