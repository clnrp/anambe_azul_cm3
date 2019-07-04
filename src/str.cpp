/*
 * str.cpp
 *
 *  Created on: 9 de set de 2018
 *      Author: cleoner
 */

#include "str.h"

int STR::Find(char *origin, char *str) // find left to right
{
	int i,j,k;
	for(i=0; origin[i]!='\0';i++){
		for(j=i,k=0; str[k]!='\0'; j++,k++){
			if(origin[j]!=str[k])
				break;
		}
		if(k>0 && str[k]=='\0')
			return i;
	}

	return -1;
}

int STR::rFind(char *origin, char *str) // find right to left
{
	int i,j,k;
	int l_ori,l_str;
    l_ori=strlen(origin); // origin text size
    l_str=strlen(str); // text size to find
	for(i=l_ori-l_str; i > 0; i--) {
		for(j=i,k=0; str[k]!='\0'; j++,k++){
			if(origin[j]!=str[k])
				break;
		}
		if(k>0 && str[k]=='\0')
			return i;
	}

	return -1;
}

int STR::Trim(char *origin, char *dest) // eliminate space, \r, \n
{
	int i,j=0;
	for(i=0; i<strlen(origin); i++) // origin text size
		if(origin[i]!=' ' && origin[i]!='\r' && origin[i]!='\n')
			dest[j++]=origin[i];
	dest[j]='\0';

    return 1;
}

int STR::Substring(char *origin, char *dest, int p1, int p2)
{
    if(p1 > p2)
    	return 0;

	memcpy(dest, origin+p1, p2-p1); // copy data from origin to dest
    dest[p2-p1]='\0';

    return 1;
}

int STR::toInt(char *str, int *value) // convert text to int
{
	int tmp=0,start=0,sign=1,size=strlen(str);

	if(!size)
		return 0;

    if(str[0]=='+' || str[0]=='-'){ // check signal
    	start=1;
    	if(str[0]=='-')
    		sign=-1;
    }

	for(int i=start, *value=0; i<strlen(str); i++){
		if(str[i]-'0' >= 0 || str[i]-'0' < 10){
			tmp += (str[i]-'0');
			if(i != size-1)
				tmp *=  10;
		}else
			return 0;
	}
	*value = sign*tmp;
	return 1;
}

int STR::toFloat(char *str, float *value) // convert text to float
{
	int point,sign=1,size=strlen(str);
	int int_value=0,fraq_value=0;
	char trim[30],buffer[20];

	if(!size)
		return 0;

	STR::Trim(str, trim);
	size=strlen(trim);

	if(trim[0]=='-')
		sign=-1;

	point=STR::Find((char*)trim,".");
	if(point != -1){
        if(point==0)
        	int_value = 0;
        else{
        	STR::Substring((char*)trim, buffer, 0, point);
        	STR::toInt(buffer, &int_value);
        }
        buffer[0]='\0';
        STR::Substring((char*)trim, buffer, point+1, size);
        STR::toInt(buffer, &fraq_value);
        float a=(float)int_value;
        float b=(float)fraq_value/(float)pow(10,size-point-1);
        *value = a+sign*b;
	}else{
		STR::toInt(trim, &int_value);
		*value = (float)int_value;
	}

	return 1;
}


int STR::hexToInt(char *str, int size)
{
	int result=0;
	for(int i=0; i<size; i++)
		result |= ascii2hex(str[i]) << 4*(size-1-i);
	return result;
}

int STR::intToHex(int value, char *result)
{
	int i;
	for(i=0; i<4; i++)
		result[i] = hex2ascii((value >> 4*(3-i))&0x0f);
	result[i] = '\0';
	return 1;
}

int STR::Split(char *buffer, char *result, char *delimit, int position)
{
	int p1=0,p2=0,size,last;
	size=strlen(buffer);
	last=STR::rFind(buffer,delimit)+strlen(delimit);

	for(int i=0; i<position && p1 <= last; i++){

		if(p1 < last){
			p2=STR::Find(buffer+p1, delimit);
			if(p2==-1)
				return 0;
			if(i+1 < size)
			    p2+=p1+strlen(delimit);

			if(i+1==position){
				STR::Substring(buffer, result, p1, p2-1);
				break;
			}
		}else{
			p2=size;
			STR::Substring(buffer, result, p1, p2);
		}
        p1=p2;
	}

	return 1;
}

