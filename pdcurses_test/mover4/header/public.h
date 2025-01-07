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

typedef struct {
        double value[10];
}data_t;

/*Prototype Area*/
void startTaskOpenGl(void);
void pthread_join_openGl(void);

#ifdef	__cplusplus
}
#endif
#endif
