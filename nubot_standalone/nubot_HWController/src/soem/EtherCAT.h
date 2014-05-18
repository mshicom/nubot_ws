#ifndef _EtherCAT_
#define _EtherCAT_

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <math.h>


#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatcoe.h"
#include "ethercatconfig.h"
#include "ethercatdc.h"

typedef struct PACKED
{
	uint16	   status1;
	uint16	   invalue1;
	uint16	   status2;
	uint16	   invalue2;
    uint16	   status3;
	uint16	   invalue3;
	uint16	   status4;
	uint16	   invalue4;
} in_EL3064t;

in_EL3064t *in_EL3064;
int os;
uint8 ob;
uint16 ob2;

void Elmo_Process_1(int32 *Speed,int32 *Speed_info);
void Elmo_Process_2(int32 *Speed);
void EtherCAT_init(const char * ifname);
void Elmo_init();
void Shoot_Control(uint8 *IsKick, uint16 ShootPower);
void Ballhandle_Enable(boolean active);
void Base_Enable(boolean active);

#endif
