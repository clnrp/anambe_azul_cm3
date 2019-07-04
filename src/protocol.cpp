/*
 * protocol.cpp
 *
 *  Created on: 18 de set de 2018
 *      Author: cleoner
 */

#include "protocol.h"
#include "global.h"

const Struct_Protocol Protocol::protocol[]={
	      {"@01", Protocol::pUpdateSetPoint},
	      {"@02", Protocol::pUpdateThro},
	      {"@03", Protocol::pUpdatePID},
		  {"@04", Protocol::pUpdateMotFactor},
	      {0,0},
};

int Protocol::process(char *buffer){

	int i=0,p=0;
	char data[100];

	while (Protocol::protocol[i].preamble){
		if (!strncmp(buffer, protocol[i].preamble, 3)){
			memcpy(data, buffer+4, strlen(buffer+4)-1);
			data[strlen(buffer+4)-1]='\0';
			return (Protocol::protocol[i].function(data));
		}
		i++;
	}

}

int Protocol::pUpdateSetPoint(char *buffer){ // update SetPoint of PID
	char  abuffer[20];
	int type;
	float value=0;

	STR::Split(buffer,abuffer,";",1); // get type
	STR::toInt(abuffer, &type);

	STR::Split(buffer,abuffer,";",2); // get setpoint value
	STR::toFloat(abuffer, &value);

	updatePidSetPoint(type, value);

	return 1;
}

int Protocol::pUpdateThro(char *buffer){ // update throttle
	float throttle;

	STR::toFloat(buffer, &throttle);

	updateThro(throttle);
	return 2;
}

int Protocol::pUpdatePID(char *buffer){ // update kp, ki and kd
	char  abuffer[20];
	int type;
	float kp=0, ki=0, kd=0;

	STR::Split(buffer,abuffer,";",1); // get type
	STR::toInt(abuffer, &type);

	STR::Split(buffer,abuffer,";",2); // get kp
	STR::toFloat(abuffer, &kp);

	STR::Split(buffer,abuffer,";",3); // get ki
	STR::toFloat(abuffer, &ki);

	STR::Split(buffer,abuffer,";",4); // get kd
	STR::toFloat(abuffer, &kd);

	updatePid(type, kp, ki, kd);

	return 1;
}

int Protocol::pUpdateMotFactor(char *buffer){ // update multiplication factor
	char  abuffer[20];
	float m1=1, m2=1, m3=1, m4=1;

	STR::Split(buffer,abuffer,";",1); // get m1
	STR::toFloat(abuffer, &m1);

	STR::Split(buffer,abuffer,";",2); // get m2
	STR::toFloat(abuffer, &m2);

	STR::Split(buffer,abuffer,";",3); // get m3
	STR::toFloat(abuffer, &m3);

	STR::Split(buffer,abuffer,";",4); // get m4
	STR::toFloat(abuffer, &m4);

	updateMotorFactor(m1, m2, m3, m4);

	return 1;
}
