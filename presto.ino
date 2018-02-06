#include "settings.h"
#include "states.h"

int track_size = 5 ; // changes depending on the track
int track[track_size] = [0,1,0,2,0]; // changes depending on the track

void setup() {
  mySetup(); //settings.h
}

void loop() {
  wait(); // states.h
  calibrate(); // states.h
  wait(); // states.h
  run(track, track_size); // states.h
}
