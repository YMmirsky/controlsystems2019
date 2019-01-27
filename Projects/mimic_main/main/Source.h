#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef PROTOPROJECT_H_
#define PROTOPROJECT_H_

#ifdef _cplusplus
extern "C" {
#endif

struct ParamsStruct {
    char name[40];
};

// this is the data struct that will be sent via. XHR to Mission Control
// Each value inside this is an angle. We will convert potentiometer values to angles.
struct ArmJointStruct {
	String rotunda;
	String shoulder;
	String elbow;
	String wrist_roll;
	String wrist_pitch;
};

void readESP32(AsyncWebServer* server, ParamsStruct* params);

bool initEEPROM();

int EEPROMCount(int addr);

void hello_world(char* name);

#ifdef _cplusplus
}
#endif

#endif

