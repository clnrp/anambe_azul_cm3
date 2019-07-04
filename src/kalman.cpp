/*
 * kalman.cpp
 *
 *  Created on: 29 de ago de 2018
 *      Author: cleoner
 */

#ifndef KALMAN_CPP_
#define KALMAN_CPP_

#include "kalman.h"

Kalman::Kalman()
{
    Q_angle = 0.001;
    Q_bias = 0.003;
    R = 0.03;
    angle = 0.0;
    bias = 0.0;

    P[0][0] = 0.0;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 0.0;
};

Kalman::Kalman(float Q_angle, float Q_bias, float R)
{
	this->Q_angle = Q_angle;
	this->Q_bias = Q_bias;
	this->R = R;
    angle = 0.0;
    bias = 0.0;

    P[0][0] = 0.0;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 0.0;
};

// https://github.com/clnrp/kfilter/blob/master/examples/ex3.py
void Kalman::Compute(float newAngle, float newVelocity, float dt)
{
	velocity = newVelocity - bias;
    angle += dt * velocity;

    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle); // Covariance matrix
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += dt * Q_bias;

    float S = P[0][0] + R; // Residue of covariance
    float K[2] = {0, 0}; // Great Kalman gain
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;
    angle += K[0] * y; // Updated status
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp; // Updated covariance
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
};

void Kalman::SetAngle(float angle)
{
	this->angle = angle;
};

float Kalman::GetAngle()
{
	return this->angle;
};

void Kalman::SetQ(float Q_angle, float Q_bias)
{
	this->Q_angle = Q_angle;
	this->Q_bias = Q_bias;
};

void Kalman::SetR(float R)
{
	this->R = R;
};

#endif /* KALMAN_CPP_ */
