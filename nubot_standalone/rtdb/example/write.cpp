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
#include "omni_ball_infor.h"

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
//   main function
// *************************
int main(void)
{
    /*int value;
    OmniBallInfor omni_ball_infor;
    omni_ball_infor.ax = omni_ball_infor.ay =
    omni_ball_infor.vx = omni_ball_infor.vy = NULL;
    //omni_ball_infor.px = omni_ball_infor.py =

	if(signal(SIGINT, signal_catch) == SIG_ERR)
	{
		printf("Error registering signal handler");
		return -1;
	}


	if(DB_init() != 0)
		return -1;

    char buffer[2];
	while(end == 0)
	{
        printf("\nInsert an integer for value: ");
		scanf("%d", &value);
		if(DB_put(VALUE, &value) == -1)
  		{
	  		DB_free();
	  		return -1;
		}

        printf("\nInsert an integer for bass pos_x: ");
        scanf("%d", &value);

        buffer[0] = static_cast<char>(value%128);
        buffer[1] = static_cast<char>(value/128);
        omni_ball_infor.px[0] = buffer[0];
        omni_ball_infor.px[1] = buffer[1];
        if(DB_put(OMNI_BALL_INFOR, &omni_ball_infor) == -1)
        {
            DB_free();
            return -1;
        }

        printf("\nInsert an integer for bass pos_y: ");
        scanf("%d", &value);
        buffer[0] = static_cast<char>(value%128);
        buffer[1] = static_cast<char>(value/128);
        omni_ball_infor.py[0] = buffer[0];
        omni_ball_infor.py[1] = buffer[1];
        if(DB_put(OMNI_BALL_INFOR, &omni_ball_infor) == -1)
        {
            DB_free();
            return -1;
        }

        printf("\nInsert an integer for bass vel_x: ");
        scanf("%d", &value);
        omni_ball_infor.vx = value;
        if(DB_put(OMNI_BALL_INFOR, &omni_ball_infor) == -1)
        {
            DB_free();
            return -1;
        }

        printf("\nInsert an integer for bass vel_y: ");
        scanf("%d", &value);
        omni_ball_infor.vy = value;
        if(DB_put(OMNI_BALL_INFOR, &omni_ball_infor) == -1)
        {
            DB_free();
            return -1;
        }
	}

    DB_free();*/

	return 0;
}
