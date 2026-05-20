#ifndef _inputHandling_Header
#define _inputHandling_Header

#include "kprint.h"

extern bool waitingForInput;
extern byte ourId;

extern bool doEcho;
extern bool doDebug;

const int inputBufferLen = 64;
const int maxTokens = 8;
extern int numTkn;

extern char inputBuffer[inputBufferLen];
extern int inputBufferPos;
extern char *tokenpointers[maxTokens]; // pointers to chars in the input buffer // Could be malloced by counting delimters


int inputHandling_loop();
int inputRoutine(); // returns 1 when input is terminated

int clearBuffer(int len);
int printBuffer();

void genTokens(char *charPntr, int len);
void printToken();
void printToken(int);

void serialFlush();

#endif