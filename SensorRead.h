#include "QTRSensors.h"
#define whiteLine 1
#define centerPosition 1000
unsigned int Sensors[NUM_SENSORS];
float errorMaximo = 0.0;// atan(Largunra do sensor/2*altura do sensor)
float lineError;
float lastRead;
int LOW_TIME = 100; //tempo minimo para repetir a checagem
int MAXCOUNT = 10; //mudar variável baseada na pista
int rightCount= 0;
int leftCount = 0;
unsigned rightValue = 0;
unsigned leftValue = 0;

QTRSensorsRC frontalSensors((unsigned char[]) {2,A5,A4,A3,A2,A1,A0,4}, NUM_SENSORS,1000);
QTRSensorsRC rightSensor((unsigned char[]) {RIGHT_SENSOR_PIN}, 1, 3000);
QTRSensorsRC leftSensor((unsigned char[]) {LEFT_SENSOR_PIN}, 1, 3000);

float errorPID(){
  
  //frontalSensors.readline (array com os pinos, modo de leitura, definição para seguir a linha braca)*
  lineError = errorMaximo*((frontalSensors.readLine(Sensors, QTR_EMITTERS_ON, whiteLine) - centerPosition)/centerPosition); // Acha o erro e cria uma média nele
  
  #ifdef DEBUG
  Serial.print("erro: ");
  Serial.println(lineError);
  for(int i = 1; i<= NUM_SENSORS; i++){
    Serial.print(sensor_value[i]);
    Serial.print("\t");
  }
  Serial.println("");
  #endif
  return lineError;
}

void readRight(){
  
  rightSensor.readCalibrated(&rightValue);

  if(millis() - lastRead > LOW_TIME){
    if(rightValue > 200){
      rightCount += 1;
      // É preciso acender o led quando notar algo?
      lastRead = millis();
    }  
  }
}

void readLeft(){
  
  leftSensor.readCalibrated(&leftValue);

  if(millis() - lastRead > LOW_TIME){
    if(leftValue > 200){
      leftCount += 1;
      // É preciso acender o led quando notar algo?
      lastRead = millis();
    } 
  }
}

float calculateError(int num_sensors, unsigned int *readings) {
  float error = 0, weight = -num_sensors/2, even = !num_sensors%2;

  for(int i = 0; i < num_sensors; i++) {
    weight++;
   
    Serial.print(readings[i]);
    Serial.print("\t");
  
    if(even && weight == 0)
      weight++;
    if(readings[i] > 200)
      error += readings[i]*weight;
         
  }

  return error/10000;
}

void calibrateSensors(QTRSensorsRC *frontal, QTRSensorsRC *left, QTRSensorsRC *right) {
  frontal->calibrate();
  left->calibrate();
  right->calibrate();
}
