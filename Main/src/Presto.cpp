//#define DEBUG
#include "Arduino.h"
#include "Pins.h"
#include "PrestoSensoring.hpp"
#include "PrestoMotorController.hpp"

#include "../lib/Definitions.h"
#include "../lib/CompilerDefinitions.h"
#include "../lib/LineFollower/LineFollower.hpp"
#include "../lib/Filter/SimpleMovingAverageFilter.hpp"


volatile bool killSwitchSignal = false;
volatile bool encoderForwardTicksCount = 0;
volatile bool encoderBackwardTicksCount = 0;

LineFollower presto;
PrestoSensoring sensoring;
Button commandButton(COMMAND_BUTTON_PIN, PULLDOWN);
SimpleMovingAverageFilter simpleMovingAverageFilter(2);

#ifdef USE_NON_LINEAR_PID
  #include "../lib/PIDController/NonLinearPIDController.hpp"
  NonLinearPIDController pidController;
#else
  #include "../lib/PIDController/PIDController.hpp"
  PIDController pidController;
#endif

PrestoMotorController motorController;

#ifdef DEBUG2
  #include "../lib/Logger/BufferLogger.hpp"
  BufferLogger logger(256);
  void commandHandlers();
#endif

void encoderInterruption();

void setup()
{
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), &encoderInterruption, CHANGE);

//#ifdef DEBUG
  Serial.begin(9600);
//#endif
  /*
  Configura a interruoção para o killSwitch.

  [TODO]
  Ainda não sabemos exatamente o que isso faz. Focamos no fato de que funciona,
  mas futuramente isso deve ser estudado.
  Muito do potencial do Arduino se perde nas limitações da bibliteca.
  */
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT0);

  sei();

  presto.setStatusPin(STATUS_LED_PIN);

  pidController.setSetPoint(0);
  pidController.setSampleTime(10);
  pidController.setOutputLimits(-255, 255);
  pidController.setControllerDirection(SystemControllerDirection::Inverse);
  pidController.setTunings(40, 22, 3);

#ifdef USE_NON_LINEAR_PID
  pidControlleir.setMaxError(10.0);
  pidController.setNonLinearConstanteCoeficients(0.5, 2, 0.5, 2, 2); // Pra ficar de acordo com os ultimos valores na antiga versão
#endif
  presto.setSystemController(&pidController);

  motorController.setPins(LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2);
  presto.setMotorController(&motorController);

  sensoring.setLineColor(LineColor::White);
  sensoring.setSensorArray(SensorArrayPins, arraySize(SensorArrayPins), 500);
  sensoring.setLeftSensor(LeftBorderSensorPin, 120, 3000);
  sensoring.setRightSensor(RightBorderSensorPin, 150, 3000);

  simpleMovingAverageFilter.setInputSource(&sensoring);
  presto.setInputSource(&simpleMovingAverageFilter);

  sensoring.calibrate(commandButton, STATUS_LED_PIN);
  presto.setLinearVelocity(120);
  presto.start();
}


void loop()
{
#ifdef DEBUG
  commandHandler();
  Serial.write(logger.getBuffer());
  logger.flush();
#endif

  /*
  Passando em 5 marcas do lado direito ou 20 segundos de prova ou o sinal
  do killswitch paramos o Presto.
  */
  if(sensoring.shouldStop(5) || presto.shouldStop(20000) || killSwitchSignal)
  {
    if(presto.getIsRunning())
    {
      presto.stop();
      Serial.println("Parou");
#ifdef DEBUG
      logger.writeLine("Fim do percurso");
#endif
    }
    else
    {
#ifdef DEBUG
      while(!commandButton.isPressed());
      //logger.writeLine("Encoder: %f.3", encoder.getTotalDistanceLeft());
      logger.writeLine("Tempo de prova: %d ms", presto.getStartTime() - presto.getStopTime());
      logger.writeLine("Kp: %f.3, Ki: %f.3, Kd: %f.3", pidController.getProportionalConstant(),
                        pidController.getIntegralConstant(), pidController.getDerivativeConstant());
      logger.writeLine("Velocidade Linear: %f.2", presto.getLinearVelocity());
      logger.writeLine("Enviando dados...");
#endif
      // Aguarda apertar de novo pra recomeçar a execução
      while(!commandButton.isPressed());

      //delay(250); // Sem esse delay, muito provavelmente ele vai pular direto pro while de calibração
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
  presto.setLinearVelocity(sensoring.inCurve() ? 60 : 120);

  presto.update();
}

ISR(PCINT0_vect)
{
  if (PINB & _BV(PB0))
  {
    Serial.println("Killswitch");
    killSwitchSignal = true;
  }
}

void encoderInterruption()
{
  if(motorController.IsLeftForward())
    encoderForwardTicksCount++;
  else
    encoderBackwardTicksCount++;
}

//#ifdef DEBUG

void commandHandler()
{
  /*
  Atenção aqui. Os comandos não podem ser muito grandes,
  caso contrário vamos estar acessando outras áreas da memória.
  Alocando isso dinâmicamente resolve o problema, mas nunca é recomendado em softwares embarcados.
  */
  char serialStream[70];
  unsigned int index = 0;
  while (Serial.available() > 0)
  {
    serialStream[index] = Serial.read();
    serialStream[++index] = '\0'; // Final do comando
  }

  // Se nada foi lido, simplesmente retorna
  if(index == 0)
    return;

  // Atenção a soma do tamanho desses arrays
  char command[50];
	char value[20];
	sscanf(serialStream, "%s %s", command, value);

  if (strcmp(command, "/kill") == 0)
  {
    killSwitchSignal = false;
  }
  else if (strcmp(command, "/setKp") == 0)
  {
    float kp = atof(value);
    float ki = pidController.getIntegralConstant();
    float kd = pidController.getDerivativeConstant();
    pidController.setTunings(kp, ki, kd);
  }
  else if (strcmp(command, "/setKi") == 0)
  {
    float kp = pidController.getProportionalConstant();
    float ki = atof(value);
    float kd = pidController.getDerivativeConstant();
    pidController.setTunings(kp, ki, kd);
  }
  else if (strcmp(command, "/setKd") == 0)
  {
    float kp = pidController.getProportionalConstant();
    float ki = pidController.getIntegralConstant();
    float kd = atof(value);
    pidController.setTunings(kp, ki, kd);
  }
  else if (strcmp(command, "/setVelocity") == 0)
  {
    presto.setLinearVelocity(atof(value));
  }
}

//#endif
