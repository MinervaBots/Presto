#ifndef Constants_h
#define Constants_h

//Constantes Do Robo
#define WHEELS_RADIUS          0.185
#define WHEELS_DISTANCE             0.143
/*
#define LARGURA_SENSOR          0.0774
#define ALTURA_SENSOR           0.120
#define TENSAO_DE_ALIMENTACAO   10.0
#define VELOCIDADE_LINEAR       12.0
#define OUT_OF_LINE             100
*/

// Sensor de reflectancia Center_position {4 sensores = 1500, 6 sensores => 2500, 8 sensores => 3500}
//#define NUM_SENSORS         4
#define CENTER_POSITION     1500.0  //não deve ser int?
#define WHITE_LINE          (unsigned char)1
#define TIMEOUT             2000.0


#endif
