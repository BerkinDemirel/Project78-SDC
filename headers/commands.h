#ifndef commands_header
#define commands_header

#include <string.h>
#include "klaus.h"

extern bool doDebug;

void PRINT(char *arg[]);
void print(char *);

extern void SETPIND(char *arg[]);  // int int
extern void SETPINA(char *arg[]);  // int int
extern void WRITED(char *arg[]);   // int int
extern void WRITEPWM(char *arg[]); // int int

extern void READPINSTATE(char *arg[]); // int  // 0-3, 0 input, 1 out, 2 inputpullup, 3 pwm
extern void READD(char *arg[]);        // int  // 0-1
extern void READA(char *arg[]);        // int  // 0-1024
extern void SETDEBUG(char *arg[]);
extern void setDebug(int state);

#ifdef acel_header
extern void ACEL(char *arg[]);
#endif

extern void THANKYOU(char *arg[]); // mp args

typedef struct CLI
{
    char *name;
    void (*func)(char *[]);
} commandType;

const commandType command[] = {
    {"print", &PRINT},          // *char[]
    {"debug", &SETDEBUG},       // bool
    {"setPIND", &SETPIND},      // int pinNum, int state
    {"writeD", &WRITED},        // int pinNum, int/bool signal
    {"writePWM", &WRITEPWM},    // int pinNum, int speed%255
    {"setPINA", &SETPINA},      // int pinNum, int state
    {"readPin", &READPINSTATE}, // int pinNum
    {"readD", &READD},          // int pinNum
    {"readA", &READA},          // int pinNum
#ifdef acel_header
    {"acel", &ACEL}, // int speed
#endif
    {"thank you", &THANKYOU},
};

const int commandCount = sizeof(command) / sizeof(commandType);

int nerveCentre(char *ptrArr[], int len); // Pass a pointer array of tokens and the amount of tokens.

#endif