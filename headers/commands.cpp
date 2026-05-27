#ifndef commands_Imp
#define commands_Imp

#include "./commands.h"
#include "hardware.h"
#include "inputHandling.h"
#include "global.h"

int nerveCentre(char *ptrArr[], int len)
{
    int result = 0;
    _DEBUG(_FF("Comparing :"))
    _DEBUG(ptrArr[0]);
    _DEBUG(_FF(": against command names."));

    if (doDebug)
    {
        koutn("Case sensitive mode :");
        if (true)
        { // todo
            kout("enabled.");
        }
        else
        {
            kout("disabled.");
        }
    }

    for (int i = 0; i < commandCount; i++)
    {
        if (doDebug)
        {
            koutn("\"");
            Serial.print(command[i].name);
            koutn("\"");
        }

        if (strcmp(command[i].name, ptrArr[0]) == 0)
        {
            if (doDebug)
            {
                kout("\t:hit.");
            }
            else if (false) // Maybe future setting for simple debug
            {
                koutn(":hit against command :");
                Serial.println(command[i].name);
            }

            command[i].func(ptrArr);
            break;
        } // end of hit

        if (doDebug)
        {
            kout("\t:miss.");
        }
    }
    return result;
}

void help()
{
    kout("TODO");
}

void HELP()
{
    kout("HELP RUNNING");
    help();
}

void print(char *pntr[])
{
    char *arg1 = *(pntr + 1);
    Serial.print(arg1);

    if (strcmp(arg1, "commands") == 0)
    {
        Serial.println();
        for (int i = 0; i < commandCount; i++)
        {
            koutn("\"");
            Serial.print(command[i].name);
            koutn("\"\t-> ");
            Serial.print((int)command[i].func);
            Serial.println();
        }
    }
    else if (strcmp(arg1, "signals") == 0)
    {
        Serial.println();
        dumpSignals();
    }
    else
    {
        for (int i = 2; i < numTkn; i++)
        {
            koutn(" ");
            Serial.print(*(pntr + i));
        }
        Serial.println();
    }
}

void PRINT(char *arg[]) // 0: 1: ...
{
    if (doDebug)
    {
        kout("PRINT RUNNING");
        print(arg);

        kout("PRINT COMPLETE");
    }
    else
    {
        print(arg);
    }
}

void SETDEBUG(char *arg[])
{
    if (doDebug)
    {
        kout("SETDEBUG RUNNING");
        setDebug(atoi(arg[1]));

        kout("SETDEBUG COMPLETE");
    }
    else
    {
    }
}
void setDebug(int state)
{
    if (doDebug)
    {
        kout("doDebug set to :0");
    }

    doDebug = state;
}

// void setpin(int pinNum, bool state)
void SETPIND(char *arg[])
{
    int pinNum = -1;
    int state = -1;

    if (doDebug)
    {
        kout("SET PIN DIGITAL RUNNING");

        koutn("arg 1\tint pinNum :");
        Serial.print(arg[1]);
        pinNum = atoi(arg[1]);

        koutn(" -> ");
        Serial.println(pinNum);

        koutn("arg 2\tint state :");
        Serial.print(arg[2]);
        state = atoi(arg[2]);

        koutn(" -> ");
        Serial.println(state);
        kout("Calling function");
    }
    else
    {
        pinNum = atoi(arg[1]);
        state = atoi(arg[2]);
    }

    setPinD(pinNum, state);
}

void SETPINA(char *arg[])
{
    int pinNum = -1;
    int state = -1;

    if (doDebug)
    {
        kout("SET PIN ANALOG RUNNING");

        koutn("arg 1\tint pinNum :");
        Serial.print(arg[1]);
        pinNum = atoi(arg[1]);
        koutn(" -> ");
        Serial.println(pinNum);

        koutn("arg 2\tbool state :");
        Serial.print(arg[2]);
        state = atoi(arg[2]);

        koutn(" -> ");
        Serial.println(state);
        kout("Calling function");
    }
    else
    {
        pinNum = atoi(arg[1]);
        state = atoi(arg[2]);
    }

    pinA[pinNum] = state;
    setPinA(pinNum, state);
}

void WRITED(char *arg[])
{
    int pinNum = -1;
    bool signal = -1;
    if (doDebug)
    {

        kout("WRITE PIN DIGITAL RUNNING");

        koutn("arg 1\tint pinNum :");
        Serial.print(arg[1]);
        pinNum = atoi(arg[1]);

        koutn(" -> ");
        Serial.println(pinNum);

        koutn("arg 2\tbool signal :");
        Serial.print(arg[2]);
        signal = (bool)atoi(arg[2]);

        koutn(" -> ");
        Serial.println(signal);
        kout("Calling function");
    }
    else
    {
        pinNum = atoi(arg[1]);
        signal = (bool)atoi(arg[2]);
    }

    writeD(pinNum, signal);
}

void WRITEA(char *arg[])
{
    kout("TODO");
}

// void setpwm(int pinNum, unsigned int speed)
void WRITEPWM(char *arg[])
{
    kout("WRITEPWM RUNNING");

    koutn("arg 1\tint pinNum :");
    Serial.print(arg[1]);
    int pinNum = atoi(arg[1]);
    koutn(" -> ");
    Serial.println(pinNum);

    koutn("arg 2\tint speed :");
    Serial.print(arg[2]);
    int speed = atoi(arg[2]);
    koutn(" -> ");
    Serial.println(speed);
    kout("Calling function");

    writePwm(pinNum, speed);
}

void READPINSTATE(char *arg[])
{ // int int
    bool isAnalog = 0;
    int pinNum = -1;

    kout("READPINSTATE RUNNING");

    koutn("arg 1\tint pinNum :");
    Serial.print(arg[1]);

    if (arg[1][0] == 'A' || arg[1][0] == 'a')
    {
        pinNum = atoi((arg[1] + 1));
        isAnalog = 1;
    }
    else
    {
        pinNum = atoi(arg[1]);
    }

    koutn(" -> ");
    Serial.println(pinNum);

    kout("Calling function");

    readPin(pinNum, isAnalog);
}

void READD(char *arg[])
{ // int
    int pinNum = 0;
    if (doDebug)
    {

        koutn("arg 1\tint pinNum :");
        Serial.print(arg[1]);
        pinNum = atoi(arg[1]);
        koutn(" -> ");
        Serial.println(pinNum);
    }

    pinNum = atoi(arg[1]);
    readD(pinNum);
}

void READA(char *arg[])
{ // int
    int pinNum = 0;
    if (doDebug)
    {
        koutn("arg 1\tint pinNum :");
        Serial.print(arg[1]);
        pinNum = atoi(arg[1]);
        koutn(" -> ");
        Serial.println(pinNum);
    }

    pinNum = atoi(arg[1]);
    readA(pinNum);
}

#ifdef acel_header
void ACEL(char *arg[])
{
    targetSpeed = atoi(arg[1]);
    _DEBUG(_FF("Printing new targetSpeed:"))
    Serial.println(targetSpeed);
    accel(targetSpeed);
}
#endif

void THANKYOU(char *arg[])
{
    kout("Aw, thanks :D");
}

#endif