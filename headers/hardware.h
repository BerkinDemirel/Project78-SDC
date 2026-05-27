#ifndef hardware_header
#define hardware_header


#include <stdlib.h>
#include "kprint.h"

extern int pinD[]; // digital pins // EEPROM
extern int pinA[];
extern int pinDsignal[];
extern int pinAsignal[];
const int writeResolution = pow(2, 8);
const int readResolution = pow(2, 10);

void hardware_init();
void hardware_loop();

void dumpSignals();

void setPinD(int pinNum, int state); // state 0 is off, 1 is output, 2 inputPullup, 3 is pwm
void setPinA(int pinNum, int state); // Not identical to setPinD, offsets

int readPin(int pinNum, bool isAnalog); // read state
int readD(int pinNum);
int readA(int pinNum);

void writeD(int pinNum, bool signal);
void writePwm(int pinNum, int speed);

void writeA(int pinNum, bool signal); // Identical to writeD except it adds the A0 offset

#endif