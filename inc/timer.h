/*
 * timer.h
 *
 *  Created on: 30 de ago de 2018
 *      Author: cleoner
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <math.h>

class TIMER{
private:
	static long int timeCount;
	static long int time0;

public:
    static void ResetTime(){
    	timeCount = 0;
    	InitRefTime();
    }

    static void Inc(){ // increment time counter
    	timeCount += 1;
    }

    static long int Get(){ // get time counter
    	return timeCount;
    }

    static void Delay_us(u32 value){ // microsecond delay
    	long int t0 = timeCount;
    	while(abs(timeCount-t0) < value){
    		asm("nop");
    	}
    }

    static void Delay_ms(u32 value){ // millisecond delay
        	long int t0 = timeCount;
        	while(abs(timeCount-t0)/100 < value){
        		asm("nop");
        	}
    }

    static void InitRefTime(){ // reference time
    	time0 = timeCount;
    }

    static float GetElapsedTime(){ // elapsed time of reference time
    	float dt = (float)(timeCount - time0)/100000.0; // us -> s
    	InitRefTime();
    	return dt;
    }
};

long int TIMER::timeCount = 0;
long int TIMER::time0 = 0;

extern "C" void TimerInc() // wrapper function
{
	TIMER::Inc();
}

void _delay_us(u32 value) // wrapper function
{
	TIMER::Delay_us(value);
}

#endif /* TIMER_H_ */
