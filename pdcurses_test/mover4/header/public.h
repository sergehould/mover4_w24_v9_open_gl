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
#ifdef	__cplusplus
extern "C" {
#endif
#include "task_controller.h"

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


#ifdef	__cplusplus
}
#endif
#endif
