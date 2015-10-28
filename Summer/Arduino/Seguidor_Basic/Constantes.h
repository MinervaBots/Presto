//#include <SoftwareSerial.h>
#include "Button.h"
#include "QTRSensors.h"
#include "Arduino.h"
#include "math.h"

//SoftwareSerial mySerial(MOSI, 4);

//Constantes Do Robo
#define RAIO_DAS_RODAS 			0.033
#define ENTRE_EIXOS				0.143
#define LARGURA_SENSOR			0.0774
#define ALTURA_SENSOR			0.120
#define TENSAO_DE_ALIMENTACAO	9.0
#define VELOCIDADE_LINEAR		2.5
//Constantes de Controle
#define KM	7.8164
#define KP	11.4 // antes: 9.0 ; teste malu: 9.8 ; 7.8
#define KI	0.04 // antes: 0.0015 ; teste malu: 0.0015 
#define KD	0.18 // antes: 0.2 ; teste malu: 0.4
#define DT  0.010
//Pinos do Microcontrolador
#define BUTTON_PIN 	5
#define R_MOTOR_1	11
#define R_MOTOR_2 3
#define L_MOTOR_1	10
#define L_MOTOR_2	9

// Sensor de reflectancia Center_positio {4 sensores = 1500, 6 sensores => 2500, 8 sensores => 3500}
#define EMITTER_PIN  		QTR_NO_EMITTER_PIN
#define NUM_SENSORS 		6
#define CENTER_POSITION  	2500.0
#define WHITE_LINE 			1
#define TIMEOUT 			2000.0

// Sensores de reflectancia laterais
#define BORDA_DIREITA
#define BORDA_ESQUERDA

// Variaveis de Debug
//#define DEBUG
//#define LOG

#define NUMBER_OF_SAMPLES 100

//Variaveis de Configuracao
//#define LEAD_LAG

	#ifdef LEAD_LAG
		#define ZERO
		#define POLE
	#endif

#define ALPHA_BETA
	
	#ifdef ALPHA_BETA
		#define ALPHA
		#define BETA
	#endif 

#define FIRST_ORDER
	#ifdef FIRST_ORDER
		#define ALPHA 0.4
	#endif
//Flags
bool following 	= true;
bool calibrando 	= true;

//Variaveis Globais
float erro_maximo 	= 10.0; //atan(LARGURA_SENSOR/(2.0 * ALTURA_SENSOR));
float theta = atan(LARGURA_SENSOR/(2.0 * ALTURA_SENSOR));
float pid_last_run	= 0.0;
float last_error 	= 0.0;
float output 		= 0.0;
float derivative = 0.0;
float integral		= 0.0;
float linear_speed	= VELOCIDADE_LINEAR;	
float angular_speed	= 0.0;
float speed_right	= 0.0;
float speed_left	= 0.0;
int   pwm_right 	= 0;
int   pwm_left		= 0;
float erro = 0.0;
float KP_nl_0 = 0.5 * KP;
float KP_nl_1 = 2 * KP;
float KD_nl_1 = 2 * KD;
float KI_nl_0 = 0.5 * KI;
float KI_nl_1 = 2 * KI;
float beta = 0.5 * KP;
float alfa0 = 0.0;
float alfa1 = 0.0;
float gama = 0.0;
float sigma = 0.0;
float KD_nl = 0.0;
float KI_nl = 0.0;
float KP_nl = 0.0;
int mark = 0;

#ifdef LOG
	float sensors_debug[NUMBER_OF_SAMPLES];
	int   aquired = 0;
#endif

#ifdef LEAD_LAG
	float lead_lag_last_input;
	float lead_lag_output;
#endif

//Classes
QTRSensorsAnalog qtra((unsigned char[]) {A4,A3,A2,A1,A0,8}, NUM_SENSORS, 1);
QTRSensorsAnalog qtrd((unsigned char[]) {BORDA_DIREITA}, 1, 1);
QTRSensorsAnalog qtre((unsigned char[]) {BORDA_ESQUERDA}, 1, 1);
Button button = Button(BUTTON_PIN,PULLDOWN);

unsigned int sensors[NUM_SENSORS];
//Funcoes
void stop();
void calculate_linear_angular_speeds();
void calculate_motor_speeds();
void calculate_pwm();
void move_robot();
float pid_control(float error);
float nonlinear_pid_control(float error);
float nonlinear2_pid_control (float error);
float lead_lag_compensator(float signal);
void store_data(float input);
void print_data();
// Funcoes Locomocao 
void stop() {
  digitalWrite(R_MOTOR_1, 1);
  digitalWrite(R_MOTOR_2, 1);
  digitalWrite(L_MOTOR_1, 1);
  digitalWrite(L_MOTOR_2, 1);
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

void calculate_pwm()
{
	pwm_right = 255 * abs(speed_right)/(KM * TENSAO_DE_ALIMENTACAO);
	pwm_right = pwm_right > 255 ? 255 : pwm_right;

	pwm_left = 255 * abs(speed_left)/(KM * TENSAO_DE_ALIMENTACAO);
	pwm_left = pwm_left > 255 ? 255 : pwm_left;
}

void move_foward()
{
  analogWrite(R_MOTOR_1,255);
  analogWrite(L_MOTOR_1,255);
  analogWrite(R_MOTOR_2,0);
  analogWrite(L_MOTOR_2,0);
}
void move_robot()
{
	if(speed_right > 0)
	{
		analogWrite(R_MOTOR_1,pwm_right);
		analogWrite(R_MOTOR_2,0);
	}
	else
	{
		analogWrite(R_MOTOR_2,pwm_right);
		analogWrite(R_MOTOR_1,0);
	}

	if(speed_left > 0)
	{
		analogWrite(L_MOTOR_1,pwm_right);
		analogWrite(L_MOTOR_2,0);
	}
	else
	{
		analogWrite(L_MOTOR_2,pwm_right);
		analogWrite(L_MOTOR_1,0);
	}
}
//Funcoes de Controle
float pid_control(float error)
{
	if(millis() - pid_last_run < DT * 1000)
		return output;
	
	pid_last_run = millis();
	
	//derivative = ALPHA * KD *(error-last_error)/DT + (1.0 - ALPHA) * derivative;
	integral += KI*DT*error;
	output = KP * error + integral + KD *(error-last_error)/DT;
	last_error = error;
	#ifdef LOG
		store_data(error);
	#endif
	return output;
}

float nonlinear_pid_control (float error)
{
  if(millis() - pid_last_run < DT * 1000)
    return output;

  pid_last_run = millis();

//constantes do PID
  alfa0 = (KP_nl_1 - KP_nl_0)/pow(erro_maximo,2);
  KP_nl_1 = alfa0 * (error*error) + beta;
  KP_nl_0 = KP_nl_1;
  
  alfa1 = KD_nl_1/(error*error);
  KD_nl_1 = alfa1 * (error*error);
  
  gama = KI_nl_0;
  sigma = log(KI_nl_1)/(log(KI_nl_0 * erro_maximo)*log(KI_nl_0 * erro_maximo));
  KI_nl_1 = gama * exp(-sigma * (gama * error)*(gama * error));

  integral += KI_nl_1 * DT * error;
  output = KP_nl_1 * error + integral + KD_nl_1*(error-last_error)/DT;
  last_error = error;
  #ifdef LOG
    store_data(error);
  #endif
    return output;
      
}
float nonlinear2_pid_control (float error)
{
  if(millis() - pid_last_run < DT * 1000)
    return output;

  pid_last_run = millis();

  alfa0 = (KP_nl_1 - KP_nl_0)/pow(erro_maximo,2);
  KP_nl = alfa0 * (error*error) + beta;
  
  alfa1 = KD_nl_1/(error*error);
  KD_nl = alfa1 * (error*error);

  gama = KI_nl_0;
  sigma = log(KI_nl_1)/(log(KI_nl_0)*(KI_nl_0*erro_maximo)*(KI_nl_0*erro_maximo));
  KI_nl = gama * exp(-sigma * (gama * error)*(gama * error) );

  integral += KI_nl * DT * error;
    
  output = KP_nl * error + integral + KD_nl*(error-last_error)/DT;
    
  last_error = error;
  #ifdef LOG
    store_data(error);
  #endif
  return output;
}
#ifdef LEAD_LAG
float lead_lag_compensator(float signal)
{
	lead_lag_output = signal - ZERO * lead_lag_last_input + POLE * lead_lag_output;
	return lead_lag_output;
}
#endif
//Estimadores
bool on_the_line()
{
  for(int i = 0; i < NUM_SENSORS; i++)
    if(sensors[i] < 500)
      return 1;
 return 0;
}
float read_sensors()
{
  return erro_maximo*((qtra.readLine(sensors, QTR_EMITTERS_ON, WHITE_LINE) - CENTER_POSITION)/CENTER_POSITION);
//  if(on_the_line())
//    erro = -  *((1000.0-sensors[0]) * 3 + (1000.0-sensors[1]) * 2 + (1000.0 -sensors[2]) - (1000.0-sensors[3]) - (1000-sensors[4]) * 2 - (1000.0-sensors[5]) *3)/12000.0; 
//  return erro;
}

float read_sensors_filtered(float alpha)
{
  erro = alpha * erro_maximo*((qtra.readLine(sensors, QTR_EMITTERS_ON, WHITE_LINE) - CENTER_POSITION)/CENTER_POSITION) + (1.0 - alpha) * erro;
  return erro; 
}

int read_border();
{
  if (qtrd.readLine(sensors, QTR_EMITTERS_ON, WHITE_LINE) && qtre.readLine(sensors, QTR_EMITTERS_ON, WHITE_LINE))
    return mark += 1;
}
void alpha_beta_filter()
{

}
void first_order()
{

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
  Serial.print("\n");
}
void print_all()
{
  Serial.print(millis());
  Serial.print(" Erro =");
  Serial.print(last_error);
  Serial.print(" Angular Speed ");
  Serial.print(angular_speed);
  Serial.print(" PWMs ");
  Serial.print(pwm_right);
  Serial.print(" ");
  Serial.print(pwm_left);
  Serial.print(" Velocidades ");
  Serial.print(speed_right);
  Serial.print(" ");
  Serial.println(speed_left); 
}
void print_time()
{
  Serial.println(millis());
}
#endif




