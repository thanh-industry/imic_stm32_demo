/*
 * iot_connect.c
 *
 *  Created on: Dec 7, 2023
 *      Author: xgene
 */

#include "string.h"
char payload[512];

char * id, *name, *depart;

void func(void)
{
	snprintf(payload, 512, "{  \
	    cardid: \"%s\", \
	    name: \"%s\", \
	    Department: \"%s\",}", id, name, depart);
}

