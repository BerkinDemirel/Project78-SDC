#ifndef hardware_Imp
#define hardware_Imp

#include <Arduino.h>
#include "global.h"
#include "hardware.h"

int pinD[14]; // digital pins. 0 is input, 1 is output, 2 is input pullup
int pinDsignal[14];
const int pinDcount = sizeof(pinD) / sizeof(pinD[0]);

int pinA[6];
int pinAsignal[6];
const int pinAcount = sizeof(pinA) / sizeof(pinA[0]);

int hardware_init()
{
    koutn("Hardware setting up :");
    Serial.print(pinDcount);
    kout(": digital pins");

    for (int i = 0; i < pinDcount; i++)
    {
        setPinD(i, pinD[i]);
    }
    kout("Digital pins ready");
    //
    koutn("Hardware, this board(arduino uno) has a analog write resolution of:");
    Serial.print(writeResolution);
    koutn(", and an analog read resolution of:");
    Serial.print(readResolution);
    kout(". These are not to be changed in this version.");

    koutn("Hardware setting up :");
    Serial.print(pinAcount);
    kout(": analog pins");

    for (int i = 0; i < pinAcount; i++)
    {
        setPinA(i, pinA[i]); // Pin A is read if no value is passed
    }
    kout("Analog pins ready");

    
}

int hardware_loop()
{
 
    // pins
    for (int i = 0; i < pinDcount; i++)
    {
        switch (pinD[i])
        {
        case 0: // input
        case 2: // input pullup
            pinDsignal[i] = digitalRead(i);
            break;
        case 1: // output
            digitalWrite(i, pinDsignal[i]);
            break;
        case 3: // pwm
                // currently run when written to
            break;
        default:
            koutn("pin :");
            Serial.print(i);
            koutn(" has faulty state :");
            Serial.println(pinD[i]);
            break;
        } // end of switch
    } // end of digital pins

    for (int i = 0; i < pinAcount; i++)
    {
        switch (pinA[i])
        {
        case 0: // input
        case 2: // input pullup
            pinAsignal[i] = analogRead(i);
            break;
        case 1: // output
            digitalWrite(i, pinAsignal[i]);
            break;
        default:
            koutn("pin :A");
            Serial.print(i);
            koutn(" has faulty state :");
            Serial.println(pinA[i]);
            break;
        } // end of switch
    }





}

void dumpSignals()
{
    kout("Digital signals");
    for (int i = 0; i < pinDcount; i++)
    {
        koutn("Pin :");
        Serial.print(i);
        koutn("\tdata direction :");
        Serial.print(pinD[i]);
        koutn("\tsignal :");
        Serial.print(pinDsignal[i]);

        if (pinD[i] == 3)
        {
            koutn("/255");
        }

        Serial.println();
    }
    //
    kout("Analog signals");
    for (int i = 0; i < pinAcount; i++)
    {
        koutn("Pin :");
        Serial.print(i);
        koutn("\tdata direction :");
        Serial.print(pinA[i]);
        koutn("\tsignal :");
        Serial.print(pinAsignal[i]);
        if (pinA[i] != 1)
        {
            koutn("/1024");
        }

        Serial.println();
    }
}

void setPinD(int pinNum, int state) // state 0 is off, 1 is output
{
    koutn("Setting digital pinNum:");
    Serial.print(pinNum);
    koutn(" to: ");
    Serial.println(state);

    pinD[pinNum] = state;
    pinDsignal[pinNum] = -1;

    pinMode(pinNum, state);
}

void setPinA(int pinNum, int state) // Abstract improvement. Just have all pins in one array and have teh pinD and PinA arrays point to it.
{
    koutn("Setting pinNum:");
    koutn("A");
    Serial.print(pinNum);

    koutn(" to: ");
    Serial.println(state);

    pinA[pinNum] = state;
    pinAsignal[pinNum] = -1; // reset the signal

    pinMode(pinNum + A0, state);
}

int readPin(int pinNum, bool isAnalog)
{
    int result = 0;
    koutn("Reading state of pin :");

    if (isAnalog)
    {
        koutn("A");
        result = pinA[pinNum];
    }
    else
    {
        result = pinD[pinNum];
    }

    Serial.println(pinNum);

    koutn("result :");
    Serial.println(result);

    return result;
}

int readD(int pinNum)
{
    int result = 0;

    if (doDebug)
    {
        koutn("Reading digital signal from pin :");
        Serial.println(pinNum);
    }

    result = digitalRead(pinNum);

    koutn("result :");
    Serial.println(result);

    return result;
}

int readA(int pinNum)
{
    int result = 0;

    _DEBUG(_FF("Reading analog signal from pin :A"));
    _DEBUG(pinNum);

    result = analogRead(pinNum + A0);

    koutn(" result :");
    Serial.println(result);

    return result;
}

void writeD(int pinNum, bool signal)
{
    if (doDebug)
    {
        koutn("Setting pinNum:");
        Serial.print(pinNum);
        koutn(" with signal :");
        Serial.print(pinDsignal[pinNum]);
        koutn(" to signal :");
        Serial.println(signal);
    }
    pinDsignal[pinNum] = signal;
}
// APplies an offset

void writePwm(int pinNum, int speed)
{
    speed = speed % writeResolution;

    if (not digitalPinHasPWM(pinNum))
    {
        koutn("Pin :");
        Serial.print(pinNum);
        koutn(" is incapable of PWM. \nExiting function.");
        Serial.println();
        return;
    }

    if (pinD[pinNum] != 3)
    {
        koutn("Pin :");
        Serial.print(pinNum);
        koutn(" is wrong state :");
        Serial.print(pinD[pinNum]);
        kout("\tExiting function.");
        return;
    }

    koutn("Setting pinNum:");
    Serial.print(pinNum);
    koutn(" with signal :");
    Serial.print(pinDsignal[pinNum]);
    koutn(" to signal :");
    Serial.print(speed);
    kout("/255");

    pinDsignal[pinNum] = speed;

    analogWrite(pinNum, speed);
}
/////

#endif