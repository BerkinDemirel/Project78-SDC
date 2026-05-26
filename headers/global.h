#ifndef _global_header
#define _global_header

#include <Arduino.h>

extern bool waitingForInput;
extern byte ourId;
extern bool doEcho;
extern bool doDebug;

#define _DEBUG(arg) if(doDebug){Serial.print(arg);}
#define _FF(arg) F(arg)
#define _NL Serial.println()

#endif