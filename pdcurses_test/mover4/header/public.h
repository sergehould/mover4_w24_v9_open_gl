 /*
 *	public.h
 * 	Contains public functions and structures.
 *	Provided to students
 *
 *	Author				Date			Version
 *	Serge Hould			26 Feb 2019		1.0.0
 *	SH					16 Dec 2020		1.0.1	Add set_warning1(char* str) and get_warning1(char* str)
 *	Serge Hould			23 Dec 2020				TESTED OK ON MOVER4 at Vanier
 *	Serge Hould			24 Dec 2020		2.0.0		Creates task_controller.h. Split public.h
 *													Move all prototypes and macros pertaining to task_controller 
 *													from public.h into task_controller.h
 *													public.h now contains only public prototypes exlcuding task_controller
  *	Serge Hould			9 Feb. 2022		2.0.1		Use of thread_cancel
 */

#ifndef PUBLIC_H
#define PUBLIC_H

#include "task_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#ifdef	__cplusplus
extern "C" {
#endif
enum { SM_PICK, SM_CARRY, SM_DROP, SM_FALL, SM_WAIT }; // for openGL state machine
#ifdef REBEL4
#define BASE_HGT 	13.43      //base height   
#define HUMERUS 	9.53      //shoulder-to-elbow "bone" 
#define ULNA		9.72      //elbow-to-wrist "bone" 
#define GRIPPER_TIP	6.000      //wrist to gripper's tip
#define GRIPPER	    5.625      //wrist to gripper’s TCP
#else
#define BASE_HGT 	8.543      //base height   
#define HUMERUS 	7.520      //shoulder-to-elbow "bone" 
#define ULNA		8.666      //elbow-to-wrist "bone" 
#define GRIPPER_TIP	6.000      //wrist to gripper's tip
#define GRIPPER	    4.800      //wrist to gripper’s TCP
#endif


typedef struct {
        double value[10];
}data_t;

/*Prototype Area*/
void set_mode(int m);
int get_mode(void);
void 	pthread_joinTimer(void);
void startTaskTimer(void);	// Ticking timer
void startTask2(void);
void pthread_join2(void);
void startTask3(void);
void pthread_join3(void);
void startTask4(void);
void pthread_join4(void);
unsigned long TickGet(void);
void startTaskTimer(void);
void pthread_joinTimer(void);
void startTask2(void);
void pthread_join2(void);
void startTask4(void);
void pthread_join4(void);
void set_mode_message(char* str);
int get_mode_message(char* str);
void startTask_test(void);
void pthread_join_test(void);

void set_state(int);
int get_state(void);
//int readADC(unsigned int);
void task_algo_cancel(void);

void startTaskAuto(void);
void startTask_display_z_r_azim(void);
void pthread_join_z_r_azim(void);
void memo_set(int m, kin_f _angles);
void jog_task_start(void);
void pthread_join_task_jog(void);
void SetMotionVec2(int i, double val);
void startTaskOpenGl(void);
void pthread_join_openGl(void);
void jog_joint(int jnt_nb, double step, double velo);
int write_file(FILE* file, double j0, double j1, double j2, double j3, double velo, double grip, double delay);

int file_read(const char* name, data_t* data);
int file_read_opened(FILE* file1, data_t* data);
int file_read_cpp(const char* name, data_t* data);

int file_line(FILE* file1);
void gl_print_warnings(double x, double y, double z);
void gl_print_errors(double x, double y, double z);



#ifdef	__cplusplus
}
#endif
#endif
