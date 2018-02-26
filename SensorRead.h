#include "QTRSensors.h"
#define whiteLine 1
#define centerPosition 3500
#define NUM_SENSORS 8
#define LINE_VALUE 500

unsigned int Sensors[NUM_SENSORS];
float errorMaximo = 0.0;// atan(Largunra do sensor/2*altura do sensor)
float lineError;
unsigned long lastReadLeft, lastReadRight;
int LOW_TIME = 400; //tempo minimo para repetir a checagem, era 100
int MAXCOUNT = 10; //mudar variável baseada na pista
int rightCount = 0;
int leftCount = 0;
unsigned rightValue = 0;
unsigned leftValue = 0;

QTRSensorsRC frontalSensors((unsigned char[]) {2, A5, A4, A3, A2, A1, A0, 4}, NUM_SENSORS, 1000);//QTRSensorsRC frontalSensors((unsigned char[]) {2,A5,A4,A3,A2,A1,A0,4}, NUM_SENSORS,1000);
QTRSensorsRC rightSensor((unsigned char[]) {RIGHT_SENSOR_PIN}, 1, 3000);
QTRSensorsRC leftSensor((unsigned char[]) {LEFT_SENSOR_PIN}, 1, 3000);

void readRight(){
  
  rightSensor.readCalibrated(&rightValue);
  if(millis() - lastReadRight > LOW_TIME)
  {
    if(rightValue < LINE_VALUE)
    {
      rightCount += 1;
      bool state = rightCount % 2 == 0 ? HIGH : LOW;
      
      if(state)
      {
 //       tone(BUZZER_PIN, 440, 50);
        digitalWrite(LED_PIN, HIGH);  
      }
      else
      {
   //     noTone(BUZZER_PIN);
        digitalWrite(LED_PIN, LOW);    
      }
      // É preciso acender o led quando notar algo?
      lastReadRight = millis();
      //Serial.println(rightValue);
    }
  }
}

void readLeft(){
  
  leftSensor.readCalibrated(&leftValue);

  if(millis() - lastReadLeft > LOW_TIME){
    if(leftValue < LINE_VALUE){
      leftCount += 1;
      // É preciso acender o led quando notar algo?
      lastReadLeft = millis();
    } 
  }
}

void calibrateSensors(QTRSensorsRC *frontal, QTRSensorsRC *right, QTRSensorsRC *left) {
  frontal->calibrate();
  left->calibrate();
  right->calibrate();
}
