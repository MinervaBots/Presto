#include "Pins.h"
#include "PrestoSensoring.hpp"
#include "CompilerDefinitions.h"
#include "PrestoMotorController.hpp"
#include "../lib/LineFollower/LineFollower.hpp"

volatile bool shouldStop;
LineFollower presto;
PrestoSensoring sensoring;
Button commandButton(COMMAND_BUTTON_PIN, PULLDOWN);

#ifdef FILTER_SAMPLES
  #include "../lib/Filter/SimpleMovingAverageFilter.hpp"
  SimpleMovingAverageFilter simpleMovingAverageFilter(FILTER_SAMPLES,
    SimpleMovingAverageFilter::FilterMode::Static);
#endif

#ifdef USE_NON_LINEAR_PID
  #include "../lib/PIDController/NonLinearPIDController.hpp"
  NonLinearPIDController pidController;
#else
  #include "../lib/PIDController/PIDController.hpp"
  PIDController pidController;
#endif

WheelEncoder encoder(0, 0); // TODO: Ver qual o pino do encoder
PrestoMotorController motorController(L_MOTOR_1_PIN, L_MOTOR_2_PIN, R_MOTOR_1_PIN, R_MOTOR_2_PIN);

#ifdef DEBUG
  BufferLogger logger(1024);
#endif


void setup()
{
#ifdef DEBUG
  Serial.begin(9600);
#endif
  attachInterrupt(digitalPinToInterrupt(KILLSWITCH_PIN), killSwitch, HIGH);

  pidController.setSetPoint(0);
  pidController.setSampleTime(10);
  pidController.setOutputLimits(-100, 100);
  pidController.setControllerDirection(SystemControllerDirection::Direct);
  pidController.setTunings(12, 40, 0.8);

#ifdef USE_NON_LINEAR_PID
  pidController.setMaxError(10.0);
  pidController.setNonLinearConstanteCoeficients(0.5, 2, 0.5, 2, 2); // Pra ficar de acordo com os ultimos valores na antiga versão
#endif
  presto.setSystemController(&pidController);

  encoder.setTicksPerRevolution(12);
  motorController.setEncoder(&encoder);
  motorController.setWheelsRadius(0.185);
  motorController.setWheelsDistance(0.143); // TODO - Medir isso novamente. Por causa do encoder, as rodas podem ficar mais proximas
  presto.setMotorController(&motorController);

  sensoring.setSensorArray(QTRSensorsRC(SensorArrayPins, sizeof(SensorArrayPins) / sizeof(char), 200));
  sensoring.setSensorLeft(QTRSensorsRC(SensorLeftBorderPins, 1, 3000));
  sensoring.setSensorRight(QTRSensorsRC(SensorRightBorderPins, 1, 3000));
  sensoring.setSampleTimes(120, 150);

#ifdef FILTER_SAMPLES
  simpleMovingAverageFilter.setInputSource(&sensoring);
  presto.setInputSource(&simpleMovingAverageFilter);
#else
  presto.setInputSource(&sensoring);
#endif

  sensoring.calibrate(commandButton, STATUS_LED_PIN);
  presto.start();
}


void loop()
{
#ifdef DEBUG
  logger.Flush(Serial.println);
#endif

  if(sensoring.shouldStop() || shouldStop)
  {
    if(presto.getIsRunning())
    {
      presto.stop();
#ifdef DEBUG
      logger.WriteLine("Fim do percurso");
#endif
    }
    else
    {
#ifdef DEBUG
      while(!commandButton.isPressed());
      logger.Write("Encoder: ");
        logger.WriteLine(encoder.getTotalDistanceLeft());

      logger.WriteLine("Enviando dados");
#endif
    }
    return;
  }

  sensoring.update();
  motorController.inCurve = sensoring.inCurve();
  presto.update();
}

void killSwitch()
{
  shouldStop = true;
}
