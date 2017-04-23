// Codigo do Seguidor sem importar funcoes externas

//#define DEBUG

#include "Constantes.h"
#include "QTRSensors.h"
#include "Button.h"

void setup() 
{

  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  
  //Calibrar
  while(!button.isPressed());
  
  delay(500);
  
  #ifdef DEBUG
   Serial.println("Butao Apertado");
  #endif
  
  digitalWrite(13,1);
  
  //Calibrando
  while(!button.isPressed())
  {
    #ifdef DEBUG
     Serial.println("calibrando");
    #endif
    qtra.calibrate();
    qtrd.calibrate();
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
 
}

void loop() 
{//Seguir
 while(following)
  {
      MAX_SPEED = 230;
      angular_speed = pid_control(read_sensors());
      //calculate_motor_speeds();
      //calculate_pwm();
      //print_each_sensor();
      //move_robot();
      move_robot_old_style(angular_speed);
      read_border();
      if (contadorSensorDireita > (num_marcacoes_pista-1)){
        following = 0;
        //angular_speed = pid_control(read_sensors());
        //move_robot_old_style(angular_speed);
        digitalWrite(13,0);   
      }
  }
  delay(150);
  stop();
  integral = 0;
  derivative = 0;
  last_error = 0;
  contadorSensorDireita = 0;
  delay(5000);
  while(!button.isPressed());
}
