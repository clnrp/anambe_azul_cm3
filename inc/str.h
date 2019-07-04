/*
 * str.h
 *
 *  Created on: 18 de set de 2018
 *      Author: cleoner
 */

#ifndef STR_H_
#define STR_H_

#include <stdio.h>
#include <string.h>
#include <math.h>

#define hex2ascii(A)  (((A)<10) ? ((A) + '0') : ((A) + 'A' - 10))
#define ascii2hex(A)  (((A)<65) ? ((A) - '0') : ((A) - 'A' + 10))

class STR{

public:
	static int Find(char *origin, char *str);
	static int rFind(char *origin, char *str);
	static int Trim(char *origin, char *dest);
	static int Substring(char *origin, char *dest, int p1, int p2);
	static int toInt(char *str, int *value);
	static int toFloat(char *str, float *value);
	static int hexToInt(char *str, int size);
	static int intToHex(int value, char *result);
	static int Split(char *buffer, char *result, char *delimit, int position);

};

#endif /* STR_H_ */
