
//valores básicos para constantes do PID
float KI = 0.00000; // constante de integração
float KP = 0.52; // 0.92 constante de proporcionalidade
float KD = 1200; // 1000 constante de derivação
float DT = 0.005; // derivação do tempo em segundos
float lastRun = 0;
float lastOutput = 0;
float MAX_OUTPUT = 1;
float MIN_OUTPUT = -1;
float setPoint = 0;
float lastError = 0;
float Integral = 0;


//função para determinar as constantes
void setConstantPID (float integrative, float proportional, float derivative,float newSetPoint){
  
  KI = integrative;
  KP = proportional;
  KD = derivative;
  setPoint = newSetPoint;
}

float controllerPID (float input)
{
  if((millis() - lastRun) < DT * 1000)
  {
    return lastOutput;
  }
  float error = setPoint - input;
  float dError = (error - lastError) / DT;  //verificar o sinal
  
  Integral += KI * error * DT;
  Integral = constrain(Integral, MIN_OUTPUT, MAX_OUTPUT);

  
  float output = (KP * error) + (Integral) + (dError * KD);
  output = constrain(output, MIN_OUTPUT, MAX_OUTPUT);
 
  lastError = error;
  lastOutput = output;
  lastRun = millis();
  return output;
}
