#include "../lib/CompilerDefinitions.h"
#include "../lib/Definitions.h"
#include "Pins.h"
#include "PrestoSensoring.hpp"
#include "PrestoMotorController.hpp"
#include "../lib/LineFollower/LineFollower.hpp"
#include "../lib/Filter/SimpleMovingAverageFilter.hpp"

volatile bool killSwitchSignal;
LineFollower presto;
PrestoSensoring sensoring;
Button commandButton(COMMAND_BUTTON_PIN, PULLDOWN);
SimpleMovingAverageFilter simpleMovingAverageFilter(5);

#ifdef USE_NON_LINEAR_PID
  #include "../lib/PIDController/NonLinearPIDController.hpp"
  NonLinearPIDController pidController;
#else
  #include "../lib/PIDController/PIDController.hpp"
  PIDController pidController;
#endif

WheelEncoder encoder(2, NOT_USED); // TODO: Ver qual o pino do encoder
PrestoMotorController motorController(L_MOTOR_1_PIN, L_MOTOR_2_PIN, R_MOTOR_1_PIN, R_MOTOR_2_PIN);

#ifdef DEBUG
  #include "../lib/Logger/BufferLogger.hpp"
  BufferLogger logger(256);
  void commandHandlers();
#endif


void setup()
{
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

#ifdef DEBUG
  Serial.begin(9600);
#endif

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

  sensoring.setSensorArray(QTRSensorsRC(SensorArrayPins, arraySize(SensorArrayPins), 200));
  sensoring.setSensorLeft(QTRSensorsRC(SensorLeftBorderPins, 1, 3000));
  sensoring.setSensorRight(QTRSensorsRC(SensorRightBorderPins, 1, 3000));
  sensoring.setSampleTimes(120, 150);

  simpleMovingAverageFilter.setInputSource(&sensoring);
  presto.setInputSource(&simpleMovingAverageFilter);

  sensoring.calibrate(commandButton, STATUS_LED_PIN);
  presto.start();
}


void loop()
{
#ifdef DEBUG
  commandHandler();
  Serial.write(logger.getBuffer());
  Serial.flush();
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
#ifdef DEBUG
      logger.writeLine("Fim do percurso");
#endif
    }
    else
    {
#ifdef DEBUG
      while(!commandButton.isPressed());
      logger.writeLine("Encoder: %f.3", encoder.getTotalDistanceLeft());
      logger.writeLine("Tempo de prova: %d ms", presto.getStartTime() - presto.getStopTime());
      logger.writeLine("Kp: %f.3, Ki: %f.3, Kd: %f.3", pidController.getProportionalConstant(),
                        pidController.getIntegralConstant(), pidController.getDerivativeConstant());
      logger.writeLine("Velocidade Linear: %f.2", presto.getLinearVelocity());
      logger.writeLine("Enviando dados...");
#endif

      // Aguarda apertar de novo pra recomeçar a execução
      while(!commandButton.isPressed());
      digitalWrite(13,1);
      delay(1000);
      killSwitchSignal = false;
      presto.start();
    }
    return;
  }

  sensoring.update();
  motorController.inCurve = sensoring.inCurve();
  presto.update();
}

ISR(PCINT0_vect)
{
  if (PINB & _BV(PB0))
    killSwitchSignal = false;
}


#ifdef DEBUG

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
  else if (strcmp(command, "/setMaxPWM") == 0)
  {
    motorController.setMaxPWM(atoi(value));
  }
  else if (strcmp(command, "/setSmoothingValue") == 0)
  {
    motorController.setActivationSmoothingValue(atoi(value));
  }
}

#endif
