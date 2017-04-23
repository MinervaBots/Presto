//Pinout do Attiny45: http://i.imgur.com/CmYEOwC.png
// A3 (Pin 3) = Porta de sinal do sensor
// A2 (Pin 4) = Porta de sinal enviado ao ATmega

void setup() {
  pinMode(A3,INPUT);
  pinMode(A2,OUTPUT);
}
int sensorValue;
void loop() {
  sensorValue=analogRead(A3);
  if(sensorValue>300){
    digitalWrite(A2,HIGH);
  }else{
    digitalWrite(A2,LOW);
  }
  //Serial.println(sensorValue);
  //delay(1);
}
