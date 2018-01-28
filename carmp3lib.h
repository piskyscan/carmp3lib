/*
 * carmp3lib.h
 *
 *  Created on: 22 Jan 2018
 *      Author: aron
 */

#ifndef CARMP3LIB_H_
#define CARMP3LIB_H_

#include <stdbool.h>


typedef void (*SUCCESSPOINT)(int address, int value, uint32_t tick, bool isRepeat, void * userData);
typedef void (*FAILPOINT)(int address, int value, uint32_t tick, int validBits, void * userData);

extern int initialise_ir_receiver(int irPort, SUCCESSPOINT successFunction, FAILPOINT failFunction, void *callerData);

extern void terminate_ir_receiver();

extern void setupCallback();

#endif /* CARMP3LIB_H_ */
