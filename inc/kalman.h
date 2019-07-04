/*
 * kalman.h
 *
 *  Created on: 29 de ago de 2018
 *      Author: cleoner
 */

#ifndef KALMAN_H_
#define KALMAN_H_

class Kalman {
public:
    Kalman();
    Kalman(float Q_angle, float Q_bias, float R);

    void Compute(float newAngle, float newSpeed, float dt);
    void SetAngle(float angle);
    float GetAngle();

    void SetQ(float Q_angle, float Q_bias);
    void SetR(float R);

private:
    float Q_angle;
    float Q_bias;
    float R;

    float angle;
    float bias;
    float velocity;

    float P[2][2];
};


#endif /* KALMAN_H_ */
