#ifndef _inputHandling_Imp
#define _inputHandling_Imp

#include "global.h"
#include "./inputHandling.h"
#include "./commands.h"

// Veriable definitions

char inputBuffer[inputBufferLen];
int inputBufferPos = 0;
char *tokenPointers[maxTokens]; // pointers to chars in the input buffer
int numTkn = 0;

int inputHandling_setup()
{
    memset(inputBuffer, 0, inputBufferLen);
    // prot
    koutn("!conreq id:");
    Serial.print(ourId);
    Serial.println();
}

void raiseWFI()
{
    if (waitingForInput == 0)
    {
        waitingForInput = 1;
        kout("!input");
    }
}

int inputHandling_loop()
{
    raiseWFI();

    if (inputRoutine())
    {
        Serial.println();
        waitingForInput = 0;

        // if (inputBuffer[0] == '\0')
        // {
        //     inputBuffer[0] = ' ';
        //     clearBuffer(1);
        //     return 1;
        // }

        if (doDebug)
        {
            printBuffer();
        }

        char *cpBuffer = (char *)malloc(inputBufferPos); // Copy a temporary buffer into memory
        memcpy(cpBuffer, inputBuffer, inputBufferPos);

        genTokens(cpBuffer, inputBufferPos);

        // command proccesing
        nerveCentre(tokenPointers, numTkn);

        free(cpBuffer);
        clearBuffer(inputBufferPos);
    }
}

int inputRoutine()
{
    int result = 0;
    if (Serial.available() > 0) // If there are any bytes in the serial.
    {
        char byte = Serial.read();

        if (doEcho && byte != '\n')
        {
            Serial.write(byte);
        }

        if (byte == '\n' || byte == '\0' || byte == '\r') // If the byte is a newline or space character complete the token and reset this buffer.
        {
            if (inputBufferPos < 1)
            {
                return 0;
            }

            byte = '\0'; // Vital
            result = 1;
        }
        else if (byte == '\b')
        {
            inputBufferPos--;
            if (doEcho)
            {
                Serial.write(" \b");
            }
            
            return result;

        } // End of checking token end

        inputBuffer[+inputBufferPos] = byte; // Put this byte in the buffer, put the next byte behind it.
        inputBufferPos++;

    } // End of if serial contains bytes
    return result;
}

int clearBuffer(int len)
{ // sets the whole buffer to 0
    for (int i = 0; i < len; i++)
    {
        inputBuffer[i] = '!' + i;
    }
    // memmove(inputBuffer,(inputBuffer + len),Serial.available()); // to , from len
    // memset(inputBuffer, 0, inputBufferPos); // Maybe later
    inputBufferPos = 0;
    // Serial.read();
    // serialFlush();
}

int printBuffer()
{
    kout("");
    koutn("Beginning to print buffer formated:");
    Serial.println(inputBuffer);
    koutn("Beginning to print buffer raw:");
    for (int i = 0; i < inputBufferLen; i++)
    {
        Serial.print(inputBuffer[i]);
    }
    kout("\nbuffer printed.");
}

void genTokens(char *charPntr, int len)
{
    if (doDebug)
    {
        Serial.println(F("starting token generation"));
    }
    char *token = strtok(charPntr, _DELLIMITER);

    numTkn = 0;
    while (token != NULL)
    {
        if (doDebug)
        {
            Serial.print(F("printing new token:"));
            koutn("numTkn");
            Serial.print(numTkn);
            koutn("=\"");
            Serial.print(token);
            kout("\"");
        }

        // Adding pointer to global array
        tokenPointers[numTkn] = token;
        // Preparing for next iteration
        token = strtok(NULL, _DELLIMITER);
        numTkn++;
    }

    if (doDebug)
    {
        koutn("stopping token generation. Ended with ");
        Serial.print(numTkn);
        kout(" token(s)");
    }
}

void printToken()
{
    for (int i = 0; i < numTkn; i++)
    {
        koutn("printing numTkn:");
        Serial.print(i);
        koutn("result :\"");
        Serial.print(tokenBuffer[i]);
        kout("\"");
    }
    // Serial.flush();
}

void printToken(int i)
{
    koutn("printing numTkn:");
    Serial.print(i);
    koutn("=\"");
    Serial.print(tokenBuffer[i]);
    kout("\"");
    // Serial.flush();
}

#endif