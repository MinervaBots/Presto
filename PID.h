
//valores básicos para constantes do PID
float KI= 0; //constante de integração
float KP = 0; //constante de proporcionalidade
float KD = 0; // constante de derivação
float DT = 0; // derivação do tempo

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
  static unsigned int lastRun = millis();
  static float lastError = error;
  
  // Isso pode causar uma divisão por zero
  float dError = -(error - lastError)/(millis() - lastRun);  //verificar o sinal
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




