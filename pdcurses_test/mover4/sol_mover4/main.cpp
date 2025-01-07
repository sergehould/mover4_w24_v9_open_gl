/****************************************************************************
 *  Renamed main.cpp
 *	Description: 
 *		Properly initializes the system and then spawns all threads beginning
 *		with controller threads.
 *		It then wait by calling join thread.
 *
 *	Author				Date			Version
 * 
 *
 *****************************************************************************/

#include <Windows.h>
#include "../header/packet.h"
#include "../header/task_controller.h"
//#include "../header/kinematic.h"
#include "../header/public.h"
#include "../header/ncurses_init.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <curses.h>
#include <stdint.h>
#include <ctype.h>
#include <errno.h>
#include <sys/types.h>
#include "../header/config.h"
#define HAVE_STRUCT_TIMESPEC  // for win32 only. Because TIMESPEC is re-defined inside pthread.h
#include <pthread.h>





int main(void)
{
	/*Ncurse config */
	ncurses_init();

	startTasksControllerRx();// combined tasks controller and received - pTask_Controller and pTask_Rx
	startTaskOpenGl();

	pthread_joinControllerRx();
	pthread_join_openGl();
	exit(EXIT_SUCCESS);
}

