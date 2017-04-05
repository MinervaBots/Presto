#include "Button.h"
#include "QTRSensors.h"
#include "Arduino.h"
#include "math.h"
#include <IRremote.h>


//Constantes Do Robo
#define RAIO_DAS_RODAS          0.185
#define ENTRE_EIXOS             0.143
#define LARGURA_SENSOR          0.0774
#define ALTURA_SENSOR           0.120
#define TENSAO_DE_ALIMENTACAO   10.0
#define VELOCIDADE_LINEAR       12.0
#define OUT_OF_LINE             100

unsigned MAX_SPEED = 90; //120 170
unsigned CURVE_SPEED = 70; //100 140
unsigned SPEED = 120; //200

//Constantes de Controle
#define KM  7.8164  //completou: 50 5 0.6, 160 curva

float KP = 40;// 27   // 25 13s -> PID: 26 0.001 0.4 ; 4 sensores ; 200 pwm ; 7.74v 
float KI = 0;// 13  // 20 0.001 - 100 - 50
float KD = 0.7;// 0.8 // 0.4 - 0.8 - 0.9

#define DT  0.005

//Pinos do Microcontrolador
#define BUTTON_PIN  5
#define R_MOTOR_1 11
#define R_MOTOR_2 3
#define L_MOTOR_1 9
#define L_MOTOR_2 10
#define SINAL_KILLSWITCH 8
#define BUZZER 12

// Sensor de reflectancia Center_position {4 sensores = 1500, 6 sensores => 2500, 8 sensores => 3500}
#define EMITTER_PIN         QTR_NO_EMITTER_PIN
#define NUM_SENSORS         4
#define CENTER_POSITION     1500.0  //não deve ser int?
#define WHITE_LINE          1
#define TIMEOUT             2000.0

// Sensores de reflectancia laterais
#define BORDA_DIREITA 7
#define BORDA_ESQUERDA  6

// Variaveis de Debug
//#define LOG

#define NUMBER_OF_SAMPLES 100  
//Flags
bool following  = true;
bool calibrando   = true;
bool ledstate = false;
bool canIread = true;

//Variaveis Globais
int num_marcacoes_pista=1;
float erro_maximo   = 10.0; //atan(LARGURA_SENSOR/(2.0 * ALTURA_SENSOR));
float theta = atan(LARGURA_SENSOR/(2.0 * ALTURA_SENSOR));
unsigned long pid_last_run  = 0;
unsigned long last_max_error  = 0;
float line_error;
float last_line_error;
float last_error  = 0.0;
float output    = 0.0;
float derivative = 0.0;
float integral    = 0.0;
float linear_speed  = VELOCIDADE_LINEAR;  

float ARW=0.0001;
float BC=0;
float angular_speed = 0.0;
float speed_right = 0.0;
float speed_left  = 0.0;
int   pwm_right   = 0;
int   pwm_left    = 0;
float erro = 0.0;
unsigned mark = 0;
int contadorSensorDireita=0;
int contadorSensorEsquerda=0;
unsigned long read_last_run = 0;
unsigned long stop_time = 0;
unsigned long start_time = 0;

//Classes
//QTRSensorsAnalog qtra((unsigned char[]) {A4,A3,A2,A1}, NUM_SENSORS, 1);
//QTRSensorsAnalog qtra((unsigned char[]) {A5,A4,A3,A2,A1,A0}, NUM_SENSORS, 1);
QTRSensorsRC qtra((unsigned char[]) {A4,A3,A2,A1}, NUM_SENSORS,200);
QTRSensorsRC qtrd((unsigned char[]) {BORDA_DIREITA}, 1, 3000);
QTRSensorsRC qtre((unsigned char[]) {BORDA_ESQUERDA}, 1, 3000);
Button button = Button(BUTTON_PIN,PULLDOWN);

unsigned int sensors[NUM_SENSORS];
//Funcoes
void stop();
void calculate_linear_angular_speeds();
void calculate_motor_speeds();
void calculate_pwm();
void move_robot(); //TALVEZ DELETAR ANALISAR DEPOIS
float pid_control(float error);
void store_data(float input);
void print_data();
void read_border();

//Necessarios ao killsiwtch
IRrecv irrecv(SINAL_KILLSWITCH);
decode_results frequenciaRecebida;
void killswitch();

// Funcoes Locomocao 
void stop() {
  analogWrite(R_MOTOR_1, 255);
  analogWrite(R_MOTOR_2, 255);
  analogWrite(L_MOTOR_1, 255);
  analogWrite(L_MOTOR_2, 255);
}

//Calcular velocidade liner e angular baseado na velocidade dos motores
void calculate_linear_angular_speeds()
{
  linear_speed = RAIO_DAS_RODAS * (speed_left + speed_right)/2.0;
  angular_speed = RAIO_DAS_RODAS * (speed_right - speed_left)/ENTRE_EIXOS;
}

//Calcula Velocidade dos 
void calculate_motor_speeds()
{
  speed_left = (2 * linear_speed - angular_speed * ENTRE_EIXOS)/(2.0 * RAIO_DAS_RODAS);
  speed_right = (2 * linear_speed + angular_speed * ENTRE_EIXOS)/(2.0 * RAIO_DAS_RODAS);
  /*Serial.print("speed_left=");
  Serial.println(speed_left);
  Serial.print("speed_right=");
  Serial.println(speed_right);*/
}

void move_foward()
{
  analogWrite(R_MOTOR_1,255);
  analogWrite(L_MOTOR_1,255);
  analogWrite(R_MOTOR_2,0);
  analogWrite(L_MOTOR_2,0);
}

void move_backward()
{
  analogWrite(R_MOTOR_1,0);
  analogWrite(L_MOTOR_1,0);
  analogWrite(R_MOTOR_2,255);
  analogWrite(L_MOTOR_2,255);
}

void move_robot_old_style(float pid_output)
{
  if(pid_output > 0){
    analogWrite(R_MOTOR_1,SPEED);
    analogWrite(R_MOTOR_2,0);
    if(SPEED - pid_output < 0){
      analogWrite(L_MOTOR_1,0);
      analogWrite(L_MOTOR_2,abs(floor(SPEED - pid_output)));
    }else{
      analogWrite(L_MOTOR_1,floor(SPEED - pid_output));
      analogWrite(L_MOTOR_2,0);
    }
  }else{
    analogWrite(L_MOTOR_1,SPEED);
    analogWrite(L_MOTOR_2,0);
    if(SPEED + pid_output < 0){
      analogWrite(R_MOTOR_1,0);
      analogWrite(R_MOTOR_2,floor(SPEED + pid_output));
    }else{
      analogWrite(R_MOTOR_1,floor(SPEED + pid_output));
      analogWrite(R_MOTOR_2,0);
    }
  }
}
//Funcoes de Controle
float pid_control(float error)
{
  if(millis() - pid_last_run < DT * 1000){
    return output;
  }  
  integral = integral + KI*DT*error;
  
  output = KP * error + integral + KD *(error-last_error)/DT;
  last_error = error;
  
  #ifdef LOG
    store_data(error);
  #endif
  
  pid_last_run = millis();
  return output;
}


float read_sensors()
{
  #ifdef DEBUG
  Serial.print("erro: ");
  Serial.print(line_error);
  Serial.print("\t");
  Serial.println("");
  #endif

  line_error = erro_maximo*((qtra.readLine(sensors, QTR_EMITTERS_ON, WHITE_LINE) - CENTER_POSITION)/CENTER_POSITION); // LINHA BRANCA
  //line_error = erro_maximo*((qtra.readLine(sensors, QTR_EMITTERS_ON) - CENTER_POSITION)/CENTER_POSITION); // LINHA PRETA
  
  return line_error;
  
  /*if(on_the_line())
    erro = -  *((1000.0-sensors[0]) * 3 + (1000.0-sensors[1]) * 2 + (1000.0 -sensors[2]) - (1000.0-sensors[3]) - (1000-sensors[4]) * 2 - (1000.0-sensors[5]) *3)/12000.0; 
  return erro;*/
}

float error(float line_error)
{
  //funçao para tratar absurdos
  if( abs(line_error)>2 && (line_error/abs(line_error))!=(last_line_error/abs(last_line_error))){
    return last_line_error;
  }
  else{
    last_line_error=line_error;
    return line_error;
  }
}


void read_right()
{
  qtrd.readCalibrated(&mark);
  
  /*#ifdef DEBUG
  Serial.println("Mark: \tContador:");
  Serial.print(mark);
  Serial.print("\t");
  Serial.println(contadorSensorDireita);
  #endif*/

  if (mark < 200) // MARCAÇÃO BRANCA // Antes o valor estava 10  27.11.2016
  //if (mark > 900) // MARCAÇÃO PRETA
  {
     if( (millis()-read_last_run) > 100){
      contadorSensorDireita = contadorSensorDireita + 1;
      digitalWrite(13,(contadorSensorDireita%2));
      read_last_run=millis();
     }
  }
}



//Função que troca o valor da variável following caso o botão do meio do controle da Sky da equipe seja apertado
void killswitch()
{
  if(irrecv.decode(&frequenciaRecebida)){ 
    if(frequenciaRecebida.value == 1336)
    {
      following = 0;
    }
    irrecv.resume(); // Receive the next value
  }
}

// Funcoes de Aquisicao de dados
#ifdef LOG
void store_data(float input)
{ 
  if(aquired < NUMBER_OF_SAMPLES - 1)
    sensors_debug[aquired++] = input;
}
void print_data()
{
  for(int i = 0; i < NUMBER_OF_SAMPLES; i++)
    Serial.println(sensors_debug[i]);
}
#endif
#ifdef DEBUG
void print_each_sensor()
{
  for(int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensors[i]);
    Serial.print("\t");
  }
  Serial.print("\t");
  //Serial.print("\n");
}
void print_all()
{
  Serial.print(millis());
  Serial.print(" Erro =");
  Serial.print(last_error);
  //Serial.print(" Angular Speed ");
  //Serial.print(angular_speed);
  Serial.print(" PWMs ");
  Serial.print(pwm_right);
  Serial.print(" ");
  Serial.print(pwm_left);
  //Serial.print(" Velocidades ");
  //Serial.print(speed_right);
  //Serial.print(" ");
  //Serial.println(speed_left); 
}
void print_time()
{
  Serial.println(millis());
}
#endif


