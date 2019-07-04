/*
 * global.h
 *
 *  Created on: 29 de ago de 2018
 *      Author: cleoner
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#ifndef VAR_GLOBAL
#define VAR_GLOBAL extern
#else
#define VAR_GLOBAL
#endif

VAR_GLOBAL int speed;
VAR_GLOBAL int cntRxBuffer;
VAR_GLOBAL char rxBuffer[200];


#endif /* GLOBAL_H_ */
