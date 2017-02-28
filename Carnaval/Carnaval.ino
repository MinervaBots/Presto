//#define DEBUG

#include "Constantes.h"
#include "QTRSensors.h"

void setup() 
{

  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  
  //Calibrar
  while(!button.isPressed());
  
  delay(500);
  
  #ifdef DEBUG
   Serial.println("Butao Apertado\nRaposo burrao");
  #endif
  
  digitalWrite(13,1);
  
  //Calibrando
  #ifdef DEBUG
   Serial.println("calibrando");
  #endif
  while(!button.isPressed())
  {
    qtra.calibrate();
    qtrd.calibrate();
    //qtre.calibrate();
  }
  //Calibrado
  #ifdef DEBUG
  Serial.println("Calibrado");
  #endif
  
  digitalWrite(13,1);
  delay(500);
  digitalWrite(13,0);
  while(!button.isPressed());
  digitalWrite(13,1);
  delay(500);
  digitalWrite(13,0);
  following = 1;
  delay(1000);
  digitalWrite(13,1);
  start_time=millis();
  pinMode(BUZZER,OUTPUT);
  pinMode(BUTTON_PIN,INPUT);
  pinMode(R_MOTOR_1,OUTPUT);
  pinMode(R_MOTOR_2,OUTPUT);
  pinMode(L_MOTOR_1,OUTPUT);
  pinMode(L_MOTOR_2,OUTPUT);
  pinMode(SINAL_KILLSWITCH,INPUT);
  pinMode(13,OUTPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  pinMode(A5,INPUT);
  pinMode(BORDA_DIREITA,INPUT);
  pinMode(BORDA_ESQUERDA,INPUT);
  irrecv.enableIRIn(); // Habilita o sensor do Killswitch
}

void loop() 
{//Seguir
 while(following)
  {
      angular_speed = pid_control(read_sensors());
      //Serial.print("pid_output ");
      //Serial.println(angular_speed);
      //print_each_sensor();
      //move_foward();
      //move_backward();
      move_robot_old_style(angular_speed);
      killswitch();
      /*if(millis()-start_time>6666){ //robô começa a ler o sensor apenas após um tempo (proximo da linha de chegada)
      read_right();
      }
      if ( contadorSensorDireita > (num_marcacoes_pista-1) ){
        following = 0;
        stop_time=millis();   
      }
      if ( (millis()-start_time) > deadline  ){
        following = 0;
        stop_time=millis();   
      }*/  
  }
  contadorSensorEsquerda=0;
  contadorSensorDireita=0;
  while( (millis()-stop_time) < 150 ){
    SPEED=160;
    angular_speed = pid_control(read_sensors());
    move_robot_old_style(angular_speed);
  }
  digitalWrite(13,0);
  #ifdef DEBUG
    Serial.println("Fim");
  #endif
  SPEED=0;
  stop();
  integral = 0;
  derivative = 0;
  last_error = 0;
  delay(150);
  while(!button.isPressed());
  digitalWrite(13,1);
  delay(1000);
  following = 1;
}
