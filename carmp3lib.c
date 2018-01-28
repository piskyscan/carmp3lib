
/*
 *
 * carmp3lib.c
 *
 * library to process carmp3 IR transmitter signals on Pi
 *
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>
#include <sys/time.h>

#ifdef USE_PIGPIO
#include <pigpio.h>
#else
#include <wiringPi.h>
#endif

#include "carmp3lib.h"



// states for receiver
enum state {
	IR_INIT,
	IR_BASE,
	IR_BIT1,
	IR_BIT2,
	IR_DATALOW,
	IR_DATAHIGH,
	IR_LASTBIT
};




typedef struct userData
{
	int state;
	int lastTick;
	int other;
	int memory[2];
	int bitsSoFar;
	int port;
	int lastFailed;

	SUCCESSPOINT successFunction;
	FAILPOINT failureFunction;
	void *callerData;

} userData;

userData memory;

void cleanup()
{
#ifdef 	USE_PIGPIO
	gpioTerminate();
#else


#endif

}

static int bigNumber = 0xfffff;
static int timeStep = 500;

void _cb(int gpio, int level, uint32_t tick, void *user)
{
	int modTick;
	int deltaT;
	int roundedT;
	int offset ;
	int bitShift;
	int mask;

	userData *dataPtr;
	dataPtr = (userData *)user;

	modTick = tick & bigNumber;

	deltaT = modTick - dataPtr->lastTick;
	// check for timer wrapping
	if (deltaT < 0)
	{
		deltaT += bigNumber;
	}

	roundedT = (deltaT+timeStep/2) /timeStep;

	dataPtr->lastTick = modTick;

	switch (dataPtr->state)
	{
	case IR_INIT:
	{
		// we set last tick
		// wait for next signal
		dataPtr->state = IR_BASE;
		dataPtr->lastFailed = true;
	}
	break;

	case IR_BASE:
	{
		// only thing to do is wait for start
		if (roundedT == 18 && level == 1)
		{
			// got start bit
			dataPtr->state = IR_BIT1;
		}
		else
		{
			// ignore
		}
	}
	break;

	case IR_BIT1:
	{
		if (roundedT == 9 && level ==0)
		{
			dataPtr->state = IR_DATALOW;

			// going to record data
			dataPtr->bitsSoFar = 0;
			dataPtr->memory[0]=0;
			dataPtr->memory[1]=0;
		}
		else
		{
			if ((roundedT == 4 || roundedT == 5) && level ==0)
			{
				if (!dataPtr->lastFailed)
				{
					// repeat
					dataPtr->successFunction(dataPtr->memory[0],dataPtr->memory[1],tick, true, dataPtr->callerData);
				}

				dataPtr->state = IR_BASE;
			}
			else
			{
				if (dataPtr->failureFunction != NULL)
				{
					dataPtr->failureFunction(dataPtr->memory[0],dataPtr->memory[1],tick, dataPtr->bitsSoFar, dataPtr->callerData);
				}

				dataPtr->state = IR_BASE;
				dataPtr->lastFailed = true;
			}
		}

	}
	break;

	case IR_BIT2:
	{
		if (roundedT ==9 && level ==0)
		{
			dataPtr->state = IR_DATALOW;
		}
		else
		{
			if (dataPtr->failureFunction != NULL)
			{
				dataPtr->failureFunction(dataPtr->memory[0],dataPtr->memory[1],tick, dataPtr->bitsSoFar, dataPtr->callerData);
			}
			dataPtr->state = IR_BASE;
			dataPtr->lastFailed = true;
		}
	}
	break;

	case IR_DATALOW:
	{
		if (roundedT ==1 && level ==1)
		{
			dataPtr->state = IR_DATAHIGH;
		}
		else
		{
			if (dataPtr->failureFunction != NULL)
			{
				dataPtr->failureFunction(dataPtr->memory[0],dataPtr->memory[1],tick, dataPtr->bitsSoFar, dataPtr->callerData);
			}
			dataPtr->state = IR_BASE;
			dataPtr->lastFailed = true;
		}
	}
	break;

	case IR_DATAHIGH:
	{
		if ((roundedT == 1 || roundedT == 3 || roundedT == 4) && level ==0)
		{
			// hurray got a bit
			if (roundedT != 1)
			{
				offset = dataPtr->bitsSoFar/16;
				bitShift = dataPtr->bitsSoFar - 16 * offset;
				mask = 1 << bitShift;
				dataPtr->state = IR_DATALOW;
				dataPtr->memory[offset] |= mask;
			}

			dataPtr->bitsSoFar += 1;
			dataPtr->state = IR_DATALOW;
			if (dataPtr->bitsSoFar == 32)
			{
				// hurray nearly done
				dataPtr->state = IR_LASTBIT;
				dataPtr->lastFailed = false;

				dataPtr->successFunction(dataPtr->memory[0],dataPtr->memory[1],tick, false, dataPtr->callerData);
			}
		}
		else
		{
			if (dataPtr->failureFunction != NULL)
			{
				dataPtr->failureFunction(dataPtr->memory[0],dataPtr->memory[1],tick, dataPtr->bitsSoFar, dataPtr->callerData);
			}

			dataPtr->state = IR_BASE;
			dataPtr->lastFailed = true;
		}
	}
	break;

	case IR_LASTBIT:
		if (roundedT == 1 && level ==1)
		{
			// complete.
			dataPtr->state = IR_BASE;
		}
		else
		{
			// well its not a disaster
			dataPtr->state = IR_BASE;
		}
		break;

	}
}


int get_useconds()
{
	int seconds,usec;

	struct timeval val;

	gettimeofday(&val, NULL);

	seconds = val.tv_sec;

	usec = val.tv_usec;
	return usec + 1000000 * seconds;
}

#ifndef USE_PIGPIO

void wiring_interupt()
{

	int usec = get_useconds();
	int value;
	value = digitalRead(memory.port);

	_cb(memory.port, value, usec, &memory);

}
#endif

void setupCallback()
{
#ifdef USE_PIGPIO
	gpioSetAlertFuncEx(memory.port , _cb, &memory);
#else
	wiringPiISR(memory.port, INT_EDGE_BOTH,  wiring_interupt);
#endif
}


int initialise_ir_receiver(int irPort, SUCCESSPOINT successFunction, FAILPOINT failFunction, void *callerData)
{
	int ret;

#ifdef 	USE_PIGPIO
	ret = gpioInitialise();

	if (ret  < 0)
	{
		fprintf(stderr, "pigpio initialisation failed with %d\n",ret);
		exit(1);
	}
#else
	wiringPiSetupGpio();
#endif


	memory.state = IR_INIT;
	memory.lastTick = 0;
	memory.port = irPort;

	atexit(&cleanup);

#ifdef 	USE_PIGPIO
	gpioSetMode(irPort, PI_INPUT);
#else
	pinMode(irPort, INPUT);
	pullUpDnControl(irPort, PUD_DOWN);

#endif

	memory.successFunction = successFunction;
	memory.failureFunction = failFunction;
	memory.callerData = callerData;

	if (successFunction == NULL)
	{
		fprintf(stderr, "cannot have null success function\n");
		exit(1);
	}

	setupCallback();
	return 0;
}

void terminate_ir_receiver()
{
	cleanup();
}
