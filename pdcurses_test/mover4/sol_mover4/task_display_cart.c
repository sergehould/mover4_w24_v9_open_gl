/*
*	Display task for z, r and azimuth
*
*	Author				Date			Version
*	Serge Hould			8 Mar.2023		v1.0.0
* *	Serge Hould			May.2023		v2.0.0	Use of of mutltiple windows - mvwprintw()
* 
*
*/


#ifdef _WIN32
#include <Windows.h>

#else
#include <unistd.h>
#include <ncurses.h>
#include <sys/ioctl.h>
#endif

#include "../header/task_controller.h"
#include "../header/public.h"
#include "../header/adc.h"
#include "../header/ncurses_init.h"
#include <math.h>	
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <curses.h>
#include <stdint.h>
#include <ctype.h>
#include <errno.h>
#include <sys/types.h>
#ifdef _WIN32
#define HAVE_STRUCT_TIMESPEC  // for win32 only. Because TIMESPEC is re-defined inside pthread.h
#endif
#include <pthread.h>

int debug = 10;


static void* pTask_Display_z_r_azim(void* ptr);

static pthread_t thread_z_r_azim;


void startTask_display_z_r_azim(void) {
	/* Thread Area	*/
	int  iret3;
	//mvwprintw(menu_win, 18,0,"Starting task 2");
	//refresh();
	/* Create independent threads each of which will execute function */
	iret3 = pthread_create(&thread_z_r_azim, NULL, pTask_Display_z_r_azim, NULL);
	if (iret3)
	{
		fprintf(stderr, "Error - pthread_create() return code: %d\n", iret3);
		exit(EXIT_FAILURE);
	}

}
void pthread_join_z_r_azim(void) {
	/* Wait till threads are complete before main continues. Unless we  */
	/* wait we run the risk of executing an exit which will terminate   */
	/* the process and all threads before the threads have completed.   */
	pthread_join(thread_z_r_azim, NULL);
}

/*
	Task Display
*/
static void* pTask_Display_z_r_azim(void* ptr) {
	kin_f curr_angles, sp_angles, cart_sp, cart_curr;
	char buf_temp[250];	// temporary buffer
	int test = adc_read(5);
	/* Menu */
//	start_color();
	init_pair(1, COLOR_BLUE, COLOR_WHITE);
	attron(COLOR_PAIR(4)); // BLUE
	mvprintw(1, 0, "*******************************************************************************");
	mvprintw(2, 0, "* Use the following keys to jog the joints or grip                            *");
	mvprintw(3, 0, "*    q         w        e       r       t                                     *");
	mvprintw(4, 0, "*  Joint1   Joint2   Joint3   Joint4  Gripper                                 *");
	mvprintw(5, 0, "*    a         s        d       f       g                                     *");
	mvprintw(6, 0, "*  Use also the following keys:                                               *");
	mvprintw(7, 0, "*  Reset errors: o - Enable Motors: m                                         *");
	mvprintw(8, 0, "*  Set joint positions to zero: z                                             *");
	mvprintw(9, 0, "*  Exit: x - Jog mode: j - Auto mode: n                                     *");
	mvprintw(10, 0, "*******************************************************************************");
	mvprintw(11, 0, "Mode: jog                                                                     ");
	mvprintw(12, 0, "                                                                              ");

	/* Setup background colour for the error/warning space*/
	mvprintw(19, 0, "                                                                               ");
	mvprintw(20, 0, "                                                                               ");
	mvprintw(21, 0, "                                                                               ");
	mvprintw(22, 0, "                                                                               ");
	mvprintw(23, 0, "                                                                               ");
	mvprintw(24, 0, "                                                                               ");
	mvprintw(25, 0, "                                                                               ");
	mvprintw(26, 0, "                                                                               ");
	attroff(COLOR_PAIR(4));
	refresh();			/* Print it on to the real screen */

	while (1) {
		//usleep(100000); // Linux
		delay_ms(200); // Windows
		sp_angles = all_sp_angles_get();
		//cart_sp = to_cart2(sp_angles);
		//curr_angles = get_all_pv_angles();
		//cart_curr = to_cart2(curr_angles);
		;
		//
		///* Prints to the console*/
		//clear();
		attron(COLOR_PAIR(3)); // GREEN
		mvprintw(13, 0, "*******************************************************************************");
		mvprintw(14, 0, "Angles:SP Bas:%7.2fdeg.  Shd:%7.2fdeg.   Elb:%7.2fdeg.   Wst:%7.2fdeg.*", sp_angle_get(0), sp_angle_get(1), sp_angle_get(2), sp_angle_get(3));
		mvprintw(15, 0, "       PV Bas:%7.2fdeg.  Shd:%7.2fdeg.   Elb:%7.2fdeg.   Wst:%7.2fdeg.*", pv_angle_get(0), pv_angle_get(1), pv_angle_get(2), pv_angle_get(3));
		mvprintw(16, 0, "                                                                         ");
		//mvprintw(17, 0, "x-y-z :SP x:%7.2fin 	  y:%7.2fin       z:%7.2fin                       *", cart_sp.data[0], cart_sp.data[1], cart_sp.data[2]);
		//mvprintw(18, 0, "x-y-z :PV x:%7.2fin 	  y:%7.2fin       z:%7.2fin                       *", cart_curr.data[0], cart_curr.data[1], cart_curr.data[2]);
		mvprintw(19, 0, "*******************************************************************************");
		attroff(COLOR_PAIR(3));

		/* If there is a warning */
		//if (get_warnings(buf_temp)==0) {
		//		attron(COLOR_PAIR(2)); // RED
		//		mvprintw(24, 0, "                                                                              ");
		//		mvprintw(24, 0, buf_temp);
		//		attroff(COLOR_PAIR(2));
		//}
		///* Erase line if no warning*/
		//else {
		//	attron(COLOR_PAIR(2)); // RED
		//	mvprintw(24, 0, "                                                                                 ");
		//	attroff(COLOR_PAIR(2));
		//}

		/* Dislpays warnings and errors*/
		//get_warnings(buf_temp);
		//get_errors(buf_temp2); // TO DO
		attron(COLOR_PAIR(2)); // RED
		mvprintw(20, 0, "                                                                                 ");
		mvprintw(21, 0, "                                                                                 ");
		//mvprintw(22, 0, buf_temp);
		//mvprintw(24, 0, buf_temp2);
		print_warnings(20, 2);
		print_errors(21, 2);

		/*Final lab exam */
		// if (get_state() == 1) mvprintw(23, 0, "moving object 1         ");
		// else if (get_state() == 2) mvprintw(23, 0, "moving object 2        ");
		// else if (get_state() == 3) mvprintw(23, 0, "returning              ");
		attroff(COLOR_PAIR(2));

		/*Mode display*/
		attron(COLOR_PAIR(4)); // Blue;
		get_mode_message(buf_temp);
		mvprintw(11, 0, "                                                                                 ");
		mvprintw(11, 0, buf_temp);
		attroff(COLOR_PAIR(4));

		// debug
		//get_test_message(buf_temp);
		//mvprintw(21, 0, "                                                                                 ");
		//mvprintw(21, 0, buf_temp);

		attron(COLOR_PAIR(3)); // GREEN;
		mvprintw(22, 0, "adc: %x                                                                                 ", readADC(5));
		print_time(23, 0);
		attroff(COLOR_PAIR(3));
		refresh();
	}

}



