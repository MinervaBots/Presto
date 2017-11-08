
//valores básicos para constantes do PID
float KI= 0; //constante de integração
float KP = 0; //constante de proporcionalidade
float KD = 0; // constante de derivação
float DT = 0; // derivação do tempo
float lastRun = 0; // ultima vez que rodou o pid
float lastError = 0; // ultimo erro
float MAX_OUTPUT = 5;
float MIN_OUTPUT = -5;
float output = 0.0;
float setPoint = 0;
float lastErro = 0;
float Integral = 0;


//função para determinar as constantes
void setConstantPID (float integrative, float proportional, float derivative,float newSetPoint){
  
  KI = integrative;
  KP = proportional;
  KD = derivative;
  setPoint = newSetPoint;
}

float controllerPID (float error){
  if(millis() - lastRun > DT*100){
    return output;
  }
  float dError = -(error - lastErro)/DT;  //verificar o sinal
  Integral += KI*DT*error;
  
  float output = KP*error;
  
  output += Integral + dError*KD;

  if (output > MAX_OUTPUT){
    output = MAX_OUTPUT;
  }
  else if (output < MIN_OUTPUT){
    output = MIN_OUTPUT;
  }
 
  lastError = error;
  lastRun = millis();
  return output;

}




