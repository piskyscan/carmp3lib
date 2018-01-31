/*
 * main.c
 *
 *  Created on: 31 Jan 2018
 *      Author: piskyscan
 *      Basic file to read IR receiver and output.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <carmp3lib.h>

#define IR_PORT 17
#define MILLION 1000000

 void IrReceive(int address, int value, uint32_t tick, bool isRepeat, void * userData)
 {

	 fprintf(stdout, "%x,%x,%d\n", value, (int)tick, (int)isRepeat);

 }



int main(int argc, char **argv)
{

    initialise_ir_receiver(IR_PORT, IrReceive, NULL, NULL);

    while (true)
    {

    	usleep(5*MILLION);
    }
}

