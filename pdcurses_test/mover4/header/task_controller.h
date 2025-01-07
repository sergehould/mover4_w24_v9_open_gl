 /*
 *	File: task_controller.h
 * 		Move all prototypes and macros pertaining to task_controller from public.h into task_controller.h
 *		public.h now contains only public prototypes exlcuding task_controller
 *
 *	Author				Date			Version
 *	Serge Hould			16 May 2022		2.0.0		
 *	Serge Hould			24 May 2022		2.0.1 add print_time() & void set_time()
 *  Serge Hould			19 March 2024	2.0.2 add data_t struct
 */

#ifndef TASK_CONTROLLER_H
#define TASK_CONTROLLER_H

#ifdef _WIN32
#include <Windows.h>
#endif
	
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <curses.h>
#include "../header/config.h"


#define JOG_MODE	0
#define NORM_MODE	1
#define TEST_MODE	2
#define RESET_ERROR		0xd0
#define EN_MOTORS		0xd1
#define RESET_MOTORS	0xd2
#define	GRIP_DIS	0x00
#ifdef  REBEL4
	#define	GRIP_CLOSE	0x02
	#define	GRIP_OPEN	0x01
#else
	#define	GRIP_CLOSE	0x02
	#define	GRIP_OPEN	0x03
#endif


#define	ONE_DEGREE	65	//joints: 1° = 65 encoder tics

 /*		Speed parameters															*/
#define 	SPEED_MAX	60.0	// high speed
//#define 	SPEED_MAX	120.0	// too much speed  - overcurrent
#define		SPEED_MED	30.0
#define		SPEED_SLOW	10.0
#define		SPEED_MIN	SPEED_SLOW
#define 	SPEED_300	180.0
#define 	SPEED_200	120.0	
#define 	SPEED_100	60.0	// high speed
#define 	SPEED_90	54.0
#define 	SPEED_80	48.0	
#define 	SPEED_70	42.0
#define 	SPEED_60	36.0
#define 	SPEED_50	30.0	
#define 	SPEED_40	24.0
#define 	SPEED_30	18.0	
#define 	SPEED_20	12.0
#define 	SPEED_10	6.0	
#define 	SPEED_5		3.0	
//#define		set_speed			set_speed_max  //an alias
#define		BLOCKING		1
#define		NON_BLOCKING	0

typedef struct
{
	double value[10];
}data_f;

typedef struct
{
	double data[6];		//angle 
}kin_f;

typedef struct
{
	int data_i[4];		// position
}kin_i;

enum { EXTRAPOL, TRAJECT, IDLE };

/*Prototype Area*/
int get_keyb_f(int s);
void set_keyb_f(int , int);
void gripper_set(int val);
int gripper_get(void);
//kin_f pv_angle_gets(void);
void clear_error(void);
void enable_motor(void);
void resetJointsToZero(void);
double pv_angle_get(int nb);
double sp_angle_get(int nb);
data_f all_sp_angles_get(void);
data_f all_pv_angles_get(void);
void startTasksControllerRx(void);
void pthread_joinControllerRx(void);
void delay_ms(int);
void set_warnings(char* str);
int get_warnings(char* str); 
void init_files(void);
void set_errors(char* str);
int get_errors(char* str);
void print_errors( int color,int v, int h);
void close_files(void);
void speed_set(double base, double shld, double elbow, double wrist);;
void print_time(int v, int h);
void set_time(int t);
void traj_set(double* ptr, int max, int blocking);
void traj_stop(void);
int readADC_udp(void);
void sp_angle_set(int nb, double value);
void print(int l, int c, char* str);
//void mvprintw_m(int l, int c, const char* format, ...);
void mvprintw_m(int col, int l, int c, const char* format, ...);
void print_warnings(int color, int v, int h);
int reset_motors(void);
int enable_motors(void);
#endif
