 /*
 *	public.h
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
#define GRIPPER	    5.000      //wrist to gripper’s TCP
#endif

#define	ERR_ROBOT		0X1
typedef struct {
        double value[10];
}data_t;

/*Prototype Area*/
void startTaskOpenGl(void);
void pthread_join_openGl(void);
void gl_print_warnings(double x, double y, double z);
void gl_print_errors(double x, double y, double z);
#ifdef	__cplusplus
}
#endif
#endif
