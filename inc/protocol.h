/*
 * protocol.h
 *
 *  Created on: 18 de set de 2018
 *      Author: cleoner
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdio.h>
#include <string.h>
#include "str.h"

typedef struct
{
	const char *preamble;
	int (*function)(char *);
}Struct_Protocol;


class Protocol{

private:
	static const Struct_Protocol protocol[];

	static int pUpdateSetPoint(char *buffer);  // update SetPoint of PID
	static int pUpdateThro(char *buffer);      // update throttle
	static int pUpdatePID(char *buffer);       // update kp, ki and kd
	static int pUpdateMotFactor(char *buffer); // update multiplication factor

public:
	static int process(char *buffer);          // process data

};

extern "C" void Process(char *buffer)          // wrapper function
{
	Protocol::process(buffer);
}

extern void updatePid(int type, float kp, float ki, float kd);
extern void updatePidSetPoint(int type, float value);
extern void updateThro(float value);
extern void updateMotorFactor(float m1, float m2, float m3, float m4);

#endif /* PROTOCOL_H_ */
