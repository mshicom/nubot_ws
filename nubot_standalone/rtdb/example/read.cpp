/*
 * Frederico Miguel Santos - frederico.miguel.santos@gmail.com
 * CAMBADA robotic soccer team - www.ieeta.pt/atri/cambada
 * University of Aveiro
 * Copyright (C) 2009
 *
 * This file is part of RTDB middleware.
 * http://code.google.com/p/rtdb/
 *
 * RTDB middleware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTDB middleware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTDB middleware.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdio.h>
#include <signal.h>
#include "rtdb_api.h"
#include "rtdb_user.h"

#include <time.h>

int end = 0;

// *************************
//   signal catch
// *************************
static void signal_catch(int sig)
{
	if (sig == SIGINT)
		end = 1;
}


// *************************
//   sleep function
// *************************
void sleep(double delay)
{
	time_t timer0, timer1;

	time(&timer0);

	do
	{
		time(&timer1);
	}while(difftime(timer1,timer0)<delay);
}



// *************************
//   main function
// *************************
int main(void)
{
	int value;

	int lifetime;

	if(signal(SIGINT, signal_catch) == SIG_ERR)
	{
		printf("Error registering signal handler");
		return -1;
	}


	if(DB_init() != 0)
		return -1;

	while(end == 0)
	{
		printf("\n\n\n");
		lifetime = DB_get(Agent0, VALUE, &value);
		printf("Value from Agent 0 = %d\t\tdata age = %dms\n", value, lifetime);
		lifetime = DB_get(Agent1, VALUE, &value);
		printf("Value from Agent 1 = %d\t\tdata age = %dms\n", value, lifetime);
		lifetime = DB_get(Agent2, VALUE, &value);
		printf("Value from Agent 2 = %d\t\tdata age = %dms\n", value, lifetime);
		sleep(0.5);
	}

	DB_free();

	return 0;
}
