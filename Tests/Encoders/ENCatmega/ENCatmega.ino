//Atenção! tirar o Serial quando implementar no código

int volatile count = 0;
void setup()
{
  pinMode(2,INPUT);
  attachInterrupt(digitalPinToInterrupt(2) , encoder , CHANGE);
  Serial.begin(9600);
}
double arc=0.875; //arco do tick em cm.
double distance=0;
void loop() {
  distance=count*arc;
  Serial.print("Distancia percorrida: ");
  Serial.print(distance);
  Serial.println("cm");
  delay(1);
}

void encoder() {
  count++;
}

