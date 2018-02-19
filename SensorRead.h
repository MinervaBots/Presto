#include "QTRSensors.h"
#define whiteLine 1
#define centerPosition 3500
#define NUM_SENSORS 8

unsigned int Sensors[NUM_SENSORS];
float errorMaximo = 0.0;// atan(Largunra do sensor/2*altura do sensor)
float lineError;
float lastRead;
int LOW_TIME = 100; //tempo minimo para repetir a checagem, era 100
int MAXCOUNT = 10; //mudar variável baseada na pista
int rightCount= 0;
int leftCount = 0;
unsigned rightValue = 0;
unsigned leftValue = 0;

QTRSensorsRC frontalSensors((unsigned char[]) {2, A5, A4, A3, A2, A1, A0, 4}, NUM_SENSORS, 1000);//QTRSensorsRC frontalSensors((unsigned char[]) {2,A5,A4,A3,A2,A1,A0,4}, NUM_SENSORS,1000);
QTRSensorsRC rightSensor((unsigned char[]) {RIGHT_SENSOR_PIN}, 1, 3000);
QTRSensorsRC leftSensor((unsigned char[]) {LEFT_SENSOR_PIN}, 1, 3000);

float errorPID(){
  
  //frontalSensors.readline (array com os pinos, modo de leitura, definição para seguir a linha braca)*
  lineError = errorMaximo*((frontalSensors.readLine(Sensors, QTR_EMITTERS_ON, whiteLine) - centerPosition)/centerPosition); // Acha o erro e cria uma média nele
  
  #ifdef DEBUG
//  Serial.print("erro: ");
//  Serial.println(lineError);
//  for(int i = 1; i<= NUM_SENSORS; i++){
//    Serial.print(sensor_value[i]);
//    Serial.print("\t");
//  }
//  Serial.println("");
  #endif
  return lineError;
}

float manualError(float error) {

/*    error = (int)
    ((4*(0 - (read(frontalSensors[0])))) + 
    (3* (0 - (read(frontalSensors[1])))) + 
    (2* (0 - (read(frontalSensors[2])))) + 
    (1* (0 - (read(frontalSensors[3])))) + 
    (1* (read(frontalSensors[4]))) + 
    (2* (read(frontalSensors[5]))) + 
    (3* (read(frontalSensors[6]))) + 
    (4* (read(frontalSensors[7]))))
*/   
    frontalSensors.read(Sensors, QTR_EMITTERS_ON);

//    Serial.println(Sensors [0]);
//    Serial.println(Sensors [1]);
 //   Serial.println(Sensors [2]);
//    Serial.println(Sensors [3]);
//    Serial.println(Sensors [4]);
//    Serial.println(Sensors [5]);
//    Serial.println(Sensors [6]);
//    Serial.println(Sensors [7]);
    /*
    error = 
    ((4*(Sensors[0]))) + 
    (3* (Sensors[1])) + 
    (2* (Sensors[2])) + 
    (1* (Sensors[3])) - 
    (1* (Sensors[4])) - 
    (2* (Sensors[5])) - 
    (3* (Sensors[6])) - 
    (4* (Sensors[7]));
    return error/10000;
    */
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
