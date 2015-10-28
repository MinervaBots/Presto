// Codigo do Seguidor sem importar funcoes externas

//#define DEBUG 

//#include <SoftwareSerial.h>
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

unsigned long begin = 0;

void loop() 
{
 // move_foward();
	//Seguir
	while(following)
	{
      if(begin == 0)
        begin = millis();
      #ifdef DEBUG 
     // print_all();
      #endif
    	angular_speed = nonlinear2_pid_control(read_sensors_filtered(0.3));
    	calculate_motor_speeds();
    	calculate_pwm();
    	move_robot();
		if (button.isPressed())
  		following = false;
    mark = read_border();
    if (mark == CHEGADA)
      following = false;   
	}

	stop();
	while(!button.isPressed());

	#ifdef LOG
		print_data();
		while(!button.isPressed());
	#endif
	
	delay(500);
	following = true;
}

