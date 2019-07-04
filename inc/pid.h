/*
 * pid.h
 *
 *  Created on: 1 de set de 2018
 *      Author: cleoner
 */

#ifndef PID_H_
#define PID_H_

#import <math.h>

class PID{
private:
	float kp;
	float ki;
	float kd;
	float setPoint;
	float out;
	float err0;
	float integral;

public:
	PID(){

	}

	PID(float kp, float ki, float kd, float setPoint){
		this->kp = kp;
		this->ki = ki;
		this->kd = kd;
		this->setPoint = setPoint;
		this->out = 0.0;
		this->err0 = 0.0;
		this->integral = 0.0;
	}

	float Compute(float value, float dt);

	float ComputeIntPid(float value, float dt);

	float GetOut();

	void SetK(float kp, float ki, float kd);

	void SetPoint(float value);

	void SetOut(float value);

};

#endif /* PID_H_ */
