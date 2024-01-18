 /*
 *	Keyboard task
 *	Given to students but only partly populated.
 *
 *	Author				Date			Version
 *	Serge Hould			26 Feb 2019		v1.0.0
 *	Serge Hould			29 March 2020	v1.1.0	Add time stamping to improve response
 *	SH					25 May 2020		Test with VB
 *	SH					29 may 2020		v1.1.1	Convert usleep() and sleep() into delay_ms()	
 *	SH					17 Dec 2020		v1.1.2  Add	 set_mode_message() and get_mode_message()
 *	Serge Hould			23 Dec 2020				TESTED OK ON MOVER4 at Vanier
 *	Serge Hould			24 Dec 2020				Add task_controller.h.
 *	Serge Hould			27 Dec 2020				Removed if(TickGet()- stamp > 20) and repaced it by delay_ms(10)
 *	Serge Hould			9 Feb. 2022		v2.0.0	Use of thread_cancel
 *	Serge Hould			27 May 2022		v3.0.0	Modify API call to set sp
 *	Serge Hould			2 Mar 2023		v3.0.1	Add flushinp()
 *  Serge Hould			9 Mar.2023		v3.0.2	set_all_sp_angles() has now a blocking parameter
 *												Add z,r azimuth control
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
#define 	JOG_ANGLE	2
#define		JOG_DELTA_Z	1
#define		JOG_DELTA_R	1



static void *pTask2( void *ptr );

static pthread_t thread2;
static pthread_mutex_t mutex_mode = PTHREAD_MUTEX_INITIALIZER;

/*Globals*/
static char buf_mode[200]; // to pass message mode

void startTask2(void){
/* Thread Area	*/
	int  iret2;
	//mvprintw(18,0,"Starting task 2");
	//refresh();
    /* Create independent threads each of which will execute function */
     iret2 = pthread_create( &thread2, NULL, pTask2, NULL);
     if(iret2)
     {
         fprintf(stderr,"Error - pthread_create() return code: %d\n",iret2);
         exit(EXIT_FAILURE);
     }

}
void pthread_join2(void){
     /* Wait till threads are complete before main continues. Unless we  */
     /* wait we run the risk of executing an exit which will terminate   */
     /* the process and all threads before the threads have completed.   */
     pthread_join( thread2, NULL);
}

/*
	Task2
	Keyboard
*/
static void *pTask2( void *ptr ){
		int input=10,i=0, mode = JOG_MODE;
		kin_f z_r_azim, sp_angles;
		double key_buff[1][4]; // a 4 angle trajectory buffer
		unsigned long stamp;	
		//Speed of the extrapolation speed of the joints
		//speed_set(SPEED_100, SPEED_100, SPEED_100, SPEED_100);	 
		speed_set(SPEED_10, SPEED_10, SPEED_10, SPEED_10);	 
//	 stamp = TickGet();
	 while(1){
		//refresh();
		flushinp(); //  discards (flushes) any characters in the input buffer associated with getch() or getchar()
		input= getchar(); // blocking with buffer - like a queue
		//input = getch(); // 
		//flushinp(); //  discards (flushes) any characters in the input buffer associated with getch() or getchar()
		delay_ms(1);
		//mvprintw(23, 15, "Keyb  ");
		//refresh();
		//delay_ms(100);
		//mvprintw(23, 15, "      ");
		//refresh();
		//delay_ms(100);
		//set_slow_degree(650*2, 650*2, 650*2, 650*2);	
		//usleep(100000);
		/* 	Executes only after 200mS (twenty 10 mS ticks)	*/
		/*	Otherwise the latest key pressed is rejected */
	//	if(TickGet()- stamp > 20){  
			//stamp = TickGet();
			switch(input){
				// reset, enable and zero
				case 'o': // Provided by the teacher
					clear_error();
					//mvprintw(20,0, "error cleared          ");
				break;
				case 'm': // Provided by the teacher	
					enable_motor();
					//mvprintw(20,0,"motor enable           ");
				break;
				case 'z': // NOT provided by the teacher
					resetJointsToZero();
					//mvprintw(20,0,"set joint pos. to zero ");
				break;
				//jog mode
				case 'j': 
					//set_mode(JOG_MODE);
					//mode = JOG_MODE;
					set_mode_message("Mode: Jog       ");
					//stop_traject();
					//task_algo_cancel();
					//attron (COLOR_PAIR (4)); // blue
					//mvprintw(13,0,"Mode: jog           ");
					//attroff(COLOR_PAIR(4));
				break;
				case 'p':
					//set_mode(TEST_MODE);
					set_mode_message("Mode: test       ");
					break;
				// normal mode
				case 'n': 
					//mode = NORM_MODE;
					set_mode_message("Mode: auto        ");
					//startTask4();
					//startTaskAuto();
					//mvprintw(13,0,"Mode: algo        ");
				break;
				case 'q': 
					//if(mode == JOG_MODE){
						//key_buff[0][0] = sp_angl_get(0)+JOG_ANGLE; // using SP
						set_mode_message("Mode: Jog       ");
						//task_algo_cancel();
						key_buff[0][0] = sp_angle_get(0) + JOG_ANGLE; // using PV
						key_buff[0][1] = sp_angle_get(1);
						key_buff[0][2] = sp_angle_get(2);
						key_buff[0][3] = sp_angle_get(3);
						traj_set(&key_buff[0][0], 1, NON_BLOCKING);
					//}
				break;
				case 'a': 
					//if(mode == JOG_MODE){
						
						key_buff[0][0] = sp_angle_get(0)-JOG_ANGLE;
						key_buff[0][1] = sp_angle_get(1);
						key_buff[0][2] = sp_angle_get(2);
						key_buff[0][3] = sp_angle_get(3);
						traj_set(&key_buff[0][0], 1, NON_BLOCKING);
					//}
				break;
				case 'w': 
					//if(mode == JOG_MODE){
						//move_joint(sp_angle_get(0) , sp_angle_get(1)+ JOG_ANGLE, sp_angle_get(2), sp_angle_get(3), 1);
						key_buff[0][0] = sp_angle_get(0);
						key_buff[0][1] = sp_angle_get(1) + JOG_ANGLE;
						key_buff[0][2] = sp_angle_get(2);
						key_buff[0][3] = sp_angle_get(3);
						traj_set(&key_buff[0][0], 1, NON_BLOCKING);
					//}
				break;
				case 's': 
					//if(mode == JOG_MODE){
						key_buff[0][0] = sp_angle_get(0);
						key_buff[0][1] = sp_angle_get(1)-JOG_ANGLE;
						key_buff[0][2] = sp_angle_get(2);
						key_buff[0][3] = sp_angle_get(3);
						traj_set(&key_buff[0][0], 1, NON_BLOCKING);
					//}
				break;
				case 'e': 
					//if(mode == JOG_MODE){
						key_buff[0][0] = sp_angle_get(0);
						key_buff[0][1] = sp_angle_get(1);
						key_buff[0][2] = sp_angle_get(2)+JOG_ANGLE;
						key_buff[0][3] = sp_angle_get(3);
						traj_set(&key_buff[0][0], 1, NON_BLOCKING);
					//}
				break;
				case 'd': 
					//if(mode == JOG_MODE){
						key_buff[0][0] = sp_angle_get(0);
						key_buff[0][1] = sp_angle_get(1);
						key_buff[0][2] = sp_angle_get(2)-JOG_ANGLE;
						key_buff[0][3] = sp_angle_get(3);
						traj_set(&key_buff[0][0], 1, NON_BLOCKING);
					//}
				break;
				case 'r': 
					//if(mode == JOG_MODE){
						key_buff[0][0] = sp_angle_get(0);
						key_buff[0][1] = sp_angle_get(1);
						key_buff[0][2] = sp_angle_get(2);
						key_buff[0][3] = sp_angle_get(3)+JOG_ANGLE;
						traj_set(&key_buff[0][0], 1, NON_BLOCKING);
					//}
				break;
				case 'f': 
					//if(mode == JOG_MODE){
						key_buff[0][0] = sp_angle_get(0);
						key_buff[0][1] = sp_angle_get(1);
						key_buff[0][2] = sp_angle_get(2);
						key_buff[0][3] = sp_angle_get(3)-JOG_ANGLE;
						traj_set(&key_buff[0][0], 1, NON_BLOCKING);
					//}
				break;
				case 't': 
					//if(mode == JOG_MODE){
						
						gripper_set(GRIP_CLOSE);
					//}
				break;
				case 'g': 
					//if(mode == JOG_MODE){
						
						gripper_set(GRIP_OPEN);
					//}
				break;
				
				case 'x': 
					endwin();
					exit(0);
				break;
			} // ends switch-case
		//} 	// end if
	} // ends loop
}


void set_mode_message(char* str) {
	pthread_mutex_lock(&mutex_mode);
	strcpy(buf_mode, str);
	pthread_mutex_unlock(&mutex_mode);
}

int get_mode_message(char* str) {
	pthread_mutex_lock(&mutex_mode);
	strcpy(str, buf_mode);
	pthread_mutex_unlock(&mutex_mode);
	if (str[0] != 0)return 0; // valid string if none null
	else return -1;
}
