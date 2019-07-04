/*
 * pid.cpp
 *
 *  Created on: 1 de set de 2018
 *      Author: cleoner
 */

#include "pid.h"

float PID::Compute(float value, float dt){
	float err = setPoint - value;
	integral += err*dt;
	float derivative = (err - err0)/dt;
	out = kp*err + ki*integral + kd*derivative;
	err0 = err;
	return out;
}

float PID::ComputeIntPid(float value, float dt){
	float err = setPoint - value;
	float integral = (err + err0);
	float derivative = (err - err0);
	out = kp*err + ki*integral + kd*derivative;
	err0 = err;
	return out;
}

float PID::GetOut(){
	return out;
}

void PID::SetK(float kp, float ki, float kd){
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

void PID::SetPoint(float value){
	this->setPoint = value;
}

void PID::SetOut(float value){
	out = value;
	integral = (out-kp*err0)/ki;
}
