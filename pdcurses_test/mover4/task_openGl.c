/* OpenGL animation of the robot arm.
* 
*	SH		22 Dec. 2023	First version
*	SH		18 Jan. 2024	Add axis, text and fine tune colors
* 
* 
*/

#include <Windows.h>
#include "header/task_controller.h"
#include "header/public.h"
#include <math.h>	
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <curses.h>
#include <stdint.h>
#include <ctype.h>
#include <errno.h>
#include <sys/types.h>
#include "header/adc.h"
#include "../freeglut/include/GL/freeglut.h"
#include "../freeglut/include/GL/glut.h"
#include "header/tick.h"


#define HAVE_STRUCT_TIMESPEC  // For win32 only - because TIMESPEC is re-defined inside pthread.h

#include <pthread.h>

/* global */

static void* pTaskOpenGl(void* ptr);
static pthread_t threadOpenGl;

long stamp;

float base = 0, shld = 0, elb = 0, wrst = 0, step = 1;
int grip= GRIP_OPEN;

#define		LENGTH		2
#define		CYL_RADIUS	0.4
#define	SCALE		0.1
#define BASE_HGT 	8.625      //base height   
#define HUMERUS 	7.375      //shoulder-to-elbow "bone" 
#define ULNA		8.625      //elbow-to-wrist "bone" 
#define GRIPPER		6.0           //gripper

#define BASE_HGT_S 	BASE_HGT*SCALE      //base height   
#define HUMERUS_S 	HUMERUS*SCALE      //shoulder-to-elbow "bone" 
#define ULNA_S		ULNA*SCALE     //elbow-to-wrist "bone" 
#define GRIPPER_S	GRIPPER*SCALE          //gripper
#define	PLANE_LENGTH	HUMERUS_S
#define	PLANE_WIDTH		0.1
#define	PLANE_DEPTH		PLANE_WIDTH

void draw_beam(float);
void x_y_z_draw(void);

// callback function
void renderScene(void){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1, 1, 1.0f, 0);
	glLoadIdentity();
	glTranslatef(0, 0, -5);
	gluLookAt(0, 0, 1, 0, 0, 0, 0.0f, 1.0f, 0.0f); //eye, center, up

	glTranslatef(0, -3*BASE_HGT_S,0); // initial y position
	x_y_z_draw();

	glColor3f(0, 0.6f, 0.0f); //green
	glRotatef(90, 1, 0, 0);
	glRotatef(-base, 0, 0, 1);
	glutWireCylinder(CYL_RADIUS, BASE_HGT_S, 20, 20);
	glRotatef(-90, 1, 0, 0);
	glTranslatef(0, 0, -BASE_HGT_S / 2);
	glTranslatef(0, CYL_RADIUS, 0);
	glutWireCylinder(CYL_RADIUS, BASE_HGT_S, 20, 20);
	glRotatef(90, 1, 0, 0);
	glRotatef(90, 0, 1, 0);
	glRotatef(-shld, 0, 1, 0);
	glTranslatef(HUMERUS_S+ CYL_RADIUS,0, 0);
	glTranslatef(0, PLANE_WIDTH/2, 0);//
	draw_beam(HUMERUS_S);

	glRotatef(-90, 1, 0, 0);
	glTranslatef(HUMERUS_S+ CYL_RADIUS  ,0 , 0);
	glTranslatef(0,0 , -PLANE_WIDTH / 2);
	glutWireCylinder(CYL_RADIUS, BASE_HGT_S, 20, 20);
	glRotatef(90, 1, 0, 0);
	glRotatef(90, 0, 1, 0);
	glTranslatef(0, BASE_HGT_S,0 );//
	glRotatef(-elb-90, 0, 1, 0);
	glTranslatef(ULNA_S + CYL_RADIUS, 0, 0);
	glTranslatef(0, -PLANE_WIDTH / 2, 0);
	draw_beam(ULNA_S);

	glRotatef(-90, 1, 0, 0);
	glTranslatef(HUMERUS_S + CYL_RADIUS , 0, 0);
	glTranslatef(0, 0, -BASE_HGT_S+ PLANE_WIDTH / 2);
	glutWireCylinder(CYL_RADIUS, BASE_HGT_S, 20, 20);
	glRotatef(90, 1, 0, 0);
	glRotatef(90, 0, 1, 0);
	glTranslatef(0, BASE_HGT_S/2, 0);//
	glRotatef(-wrst - 90, 0, 1, 0);
	glTranslatef(GRIPPER_S + CYL_RADIUS, 0, 0);

	/* Draws the gripper */
	if (grip ==GRIP_CLOSE) { 
		glTranslatef(0, PLANE_WIDTH*1.5, 0);
		draw_beam(GRIPPER_S);
		glTranslatef(0, -PLANE_WIDTH * 1.5*2, 0);
		draw_beam(GRIPPER_S);
	}
	else {
		glTranslatef(0, -BASE_HGT_S/2+ PLANE_WIDTH, 0);
		draw_beam(GRIPPER_S);
		glTranslatef(0, +BASE_HGT_S - PLANE_WIDTH*2, 0);
		draw_beam(GRIPPER_S);
	}

	if (tick_diff(stamp) >= 10) {
		base = pv_angle_get(0);
		shld =pv_angle_get(1);
		elb = pv_angle_get(2);
		wrst = pv_angle_get(3);
		stamp = tick_get();
		grip = gripper_get();
	}
	glutSwapBuffers();
}

void changeSize(int w, int h){
	float ratio = w * 1.0 / h;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);
	gluPerspective(90, ratio, 1, 100);
	glMatrixMode(GL_MODELVIEW);
}

//void pressNormalKeys(unsigned char key, int x, int y) {
//	if (key == 'w') {
//		step += 1;
//	}
//
//	if (key == 's') {
//		step -= 1;
//	}
//}


void startTaskOpenGl(void) {
	/* Thread Area	*/
	int  iret4;
	iret4 = pthread_create(&threadOpenGl, NULL, pTaskOpenGl, NULL);
	if (iret4)
	{
		fprintf(stderr, "Error - pthread_create() return code: %d\n", iret4);
		exit(EXIT_FAILURE);
	}

}
void pthread_join_openGl(void) {
	/* Wait till threads are complete before main continues. Unless we  */
	/* wait we run the risk of executing an exit which will terminate   */
	/* the process and all threads before the threads have completed.   */
	pthread_join(threadOpenGl, NULL);
}


/*
	TaskOpenGL
	Animation
*/
static void* pTaskOpenGl(void* ptr) {
	//make it cancellable
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
	int argc=0;
	char** argv = 0;
	//printf("thread called\n");
	tick_init();
	stamp = tick_get();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(800, 10);
	glutInitWindowSize(640, 640);
	glutCreateWindow("Mover4");
	glutDisplayFunc(renderScene);
	glEnable(GL_DEPTH_TEST); // to avoid back get drawn to the display over the top of the model at the front
	glutIdleFunc(renderScene);
	glutReshapeFunc(changeSize);
	//glutKeyboardFunc(pressNormalKeys);
	glutMainLoop();
	return 0;
}

/* draws a beam */
void draw_beam(float length) {
	/* 3d plane */
	glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
	glColor3f(0.0f, 0.8f, 0.0f);     // Green

	glVertex3f(length, PLANE_WIDTH, -PLANE_DEPTH);
	glVertex3f(-length, PLANE_WIDTH, -PLANE_DEPTH);
	glVertex3f(-length, PLANE_WIDTH, PLANE_DEPTH);
	glVertex3f(length, PLANE_WIDTH, PLANE_DEPTH);

	// Bottom face (y = -1.0f)
	glColor3f(0.0f, 1.0f, 0.0f);     // green
	glVertex3f(length, -PLANE_WIDTH, PLANE_DEPTH);
	glVertex3f(-length, -PLANE_WIDTH, PLANE_DEPTH);
	glVertex3f(-length, -PLANE_WIDTH, -PLANE_DEPTH);
	glVertex3f(length, -PLANE_WIDTH, -PLANE_DEPTH);

	// Front face  (z = 1.0f)
	glColor3f(0.0f, 0.7f, 0.0f);     // 
	glVertex3f(length, PLANE_WIDTH, PLANE_DEPTH);
	glVertex3f(-length, PLANE_WIDTH, PLANE_DEPTH);
	glVertex3f(-length, -PLANE_WIDTH, PLANE_DEPTH);
	glVertex3f(length, -PLANE_WIDTH, PLANE_DEPTH);

	// Back face (z = -1.0f)
	glColor3f(0.0f, 0.4f, 0.0f);     // Green
	glVertex3f(length, -PLANE_WIDTH, -PLANE_DEPTH);
	glVertex3f(-length, -PLANE_WIDTH, -PLANE_DEPTH);
	glVertex3f(-length, PLANE_WIDTH, -PLANE_DEPTH);
	glVertex3f(length, PLANE_WIDTH, -PLANE_DEPTH);

	// Left face (x = -1.0f)
	glColor3f(0.0f, 0.5f, 0.0f);     // Green
	glVertex3f(-length, PLANE_WIDTH, PLANE_DEPTH);
	glVertex3f(-length, PLANE_WIDTH, -PLANE_DEPTH);
	glVertex3f(-length, -PLANE_WIDTH, -PLANE_DEPTH);
	glVertex3f(-length, -PLANE_WIDTH, PLANE_DEPTH);

	// Right face (x = 1.0f)
	glColor3f(0.0f, 0.6f, 0.0f);     // Green
	glVertex3f(length, PLANE_WIDTH, -PLANE_DEPTH);
	glVertex3f(length, PLANE_WIDTH, PLANE_DEPTH);
	glVertex3f(length, -PLANE_WIDTH, PLANE_DEPTH);
	glVertex3f(length, -PLANE_WIDTH, -PLANE_DEPTH);
	glEnd();  // End of drawing color-cube

}

/* Draws the z-y-z axis*/
void x_y_z_draw(void) {
	glColor3f(1, 0, 0); //red
	glLineWidth(1);
	//z axis
	glBegin(GL_LINES);
	glVertex3f(-HUMERUS_S * 4, -BASE_HGT_S, 0);
	glVertex3f(-HUMERUS_S * 4, HUMERUS_S * 2 - BASE_HGT_S, 0);
	glEnd();
	// z letter
	glRasterPos3f((-HUMERUS_S * 4), (HUMERUS_S * 2 - BASE_HGT_S) + (HUMERUS_S * 2 - BASE_HGT_S) * 0.2, 0);// position
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'Z');

	//y axis
	glBegin(GL_LINES);
	glVertex3f(-HUMERUS_S * 4, -BASE_HGT_S, 0);
	glVertex3f(-HUMERUS_S * 4, -BASE_HGT_S, -HUMERUS_S * 2);
	glEnd();

	// y letter
	glRasterPos3f((-HUMERUS_S * 4), -BASE_HGT_S, (-HUMERUS_S * 2) + (-HUMERUS_S * 2) * 0.2);// position
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'Y');

	//x axis
	glBegin(GL_LINES);
	glVertex3f(-HUMERUS_S * 4, -BASE_HGT_S, 0);
	glVertex3f(-HUMERUS_S * 2, -BASE_HGT_S, 0);
	glEnd();

	// x letter
	glRasterPos3f((-HUMERUS_S * 2) + (HUMERUS_S * 2) * 0.2, -BASE_HGT_S, 0);// position
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'X');

}
