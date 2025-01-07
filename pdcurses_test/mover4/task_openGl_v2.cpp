/*
*	SH		10 May 2024		New version with lighting
*	SH		3 June 2024		Add vial, camera and more color.
* 	SH		6 June 2024		Almost completed
*
*/

#include <Windows.h>
#include <iostream>
#include <vector>
#include "header/task_controller.h"
//#include "header/kinematic.h"
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
#include "header/public.h"

#define HAVE_STRUCT_TIMESPEC  // For win32 only - because TIMESPEC is re-defined inside pthread.h

#include <pthread.h>

/* Color sets */
static GLfloat black[] = { 0.0, 0.0, 0.0, 1.0 };
static GLfloat pale_blue[] = { 0.68f, 0.85f, 0.90f, 1.0f };
static GLfloat sky_blue[] = { 0.53f, 0.81f, 0.92f, 1.0f };
static GLfloat marine_blue[] = { 0.00f, 0.50f, 0.50f, 1.0f };
static GLfloat yellow[] = { 1.0, 1.0, 0.0, 1.0 };
static GLfloat cyan[] = { 0.0, 1.0, 1.0, 1.0 };
static GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat red[] = { 1.0, 0.0, 0.0, 1.0 };
static GLfloat green[] = { 0.0, 1.0, 0.0, 1.0 };
static GLfloat grey[] = { 0.752941, 0.752941, 0.752941, 1.0 };
static GLfloat orange2[] = { 0.89, 0.47, 0.20, 1.0 };
static GLfloat orange[] = { 1.0, 0.50, 0.0, 1.0 };

#define	SCALE		0.1
#define	MOTOR_CYL_HGT		(BASE_HGT*0.75) //motor body height 
#define	MOTOR_CYL_RADIUS	(BASE_HGT*0.25)  // motor body diameter 
#define	ARM_CYL_RADIUS		(MOTOR_CYL_RADIUS*0.4) // arm diameter proportional to motor
#define	GRIP_CYL_RADIUS		(MOTOR_CYL_RADIUS*0.2) // grip diameter proportional to motor

#define BASE_HGT_S 			(BASE_HGT*SCALE)		//base height   
#define HUMERUS_S 			(HUMERUS*SCALE)			//shoulder-to-elbow "bone" 
#define ULNA_S				(ULNA*SCALE)			//elbow-to-wrist "bone" 
#define GRIPPER_S			(GRIPPER_TIP*SCALE)          //gripper tip
#define GRIPPER_TCP_S		(GRIPPER*SCALE)          //gripper tcp
#define GRIP_CYL_RADIUS_S	(GRIP_CYL_RADIUS*SCALE)

#define	MOTOR_CYL_HGT_S		(MOTOR_CYL_HGT*SCALE)
#define	MOTOR_CYL_RADIUS_S	(MOTOR_CYL_RADIUS*SCALE)
#define	ARM_CYL_RADIUS_S	(ARM_CYL_RADIUS*SCALE)

#define	TOP					((BASE_HGT + HUMERUS + ULNA + GRIPPER_TIP)*SCALE) // z tip of the robot when vertically positioned

/* Vial dimensions */
//#define VIAL_RADIUS		0.75
//#define	VIAL_HGT		3.0	
#define VIAL_RADIUS		0.75
#define	VIAL_HGT		3.0	
#define VIAL_RADIUS_S	VIAL_RADIUS*SCALE
#define	VIAL_HGT_S		VIAL_HGT*SCALE

/* Pedestal dimensions */
//#define PED_RADIUS		1
//#define	PED_HGT			4	
#define PED_RADIUS		1
#define	PED_HGT			4	
#define PED_RADIUS_S	PED_RADIUS*SCALE
#define	PED_HGT_S		PED_HGT*SCALE	

#define ROBOT_COLOUR	white
#define TABLE_COLOUR	marine_blue


#define PATH_MAX		2000
// Define a structure to represent a 3D point
typedef struct {
	double x;
	double y;
	double z;
}Point3D;
//enum { E_GREEN, E_RED, E_YELLOW };

/* Global variables */
int state = SM_PICK; // shared with adc.cpp

/* Local variables */
Point3D vial_center, tcp_center, fall_center, tcp_center_previous;
Point3D tcp_array[PATH_MAX];
static pthread_t threadOpenGl;
static data_t line1, line2;
static long stamp, stamp2;
static float base = 0, shld = 0, elb = 0, wrst = 0, step = 1;
static int grip = GRIP_OPEN;
static double theta = 90 * 3.1416 / 180;      // determines the x and z positions
static double y = 0;          // the current y position
static double dTheta = 0.04;     // increment in theta for swinging the camera around
static double dy = 0.2;         // increment in y for moving the camera up/down
static double eye_x = 0, eye_y = 0, eye_z = 1;
static data_f angle;
double distance, grip_to_hor;
int touched_f = 0;
int i = 0, debug1, path_max=0;








/* Camera movement with the arrows */
static void moveRight() { theta += dTheta; }
static void moveLeft() { theta -= dTheta; }
static void moveUp() { if (y < BASE_HGT_S*10) y += dy; }
//void moveDown() { if (y > dy) y -= dy; }
static void moveDown() { if (y > -BASE_HGT_S) y -= dy; }
static double getX() { return TOP * cos(theta); }
static double getY() { return y; }
static double getZ() { return TOP * sin(theta); }
static void* pTaskOpenGl(void* ptr);
static int file_read_(const char* name, data_t* data);
static Point3D to_cart2(data_f angle);
static void x_y_z_draw(void);
static double distance_3d(Point3D point1, Point3D point2);
//void glPrint(double x, double y, double z);

#define	Z_OFFSET	-TOP*0.333

std::vector<Point3D> dynamic_tcp_array(1); // Create an empty dynamic array (vector)

void drawTable(float step) {
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, TABLE_COLOUR);

	// Draw  lines
	int start = (int)TOP;
	// Draw cells 
	//glBegin(GL_QUADS);

	//for (float i = -start; i <= start; i = i + step) {
	//	for (float j = -start; j <= start; j = j + step) {
	//		glVertex3f(i, 0.0f, j);
	//		glVertex3f(i + 1, 0.0f, j); // y-z-x
	//		glVertex3f(i + 1, 0.0f, j+1);
	//		glVertex3f(i, 0.0f, j+1);
	//	}
	//}
	//glEnd();

	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
	glLineWidth(1.0f);
	glBegin(GL_LINES);
	for (float i = -start; i <= start; i= i+step) {
		glVertex3f(start, 0, i); // y-z-x
		glVertex3f(-start, 0, i);
	}
	// Draw perpendicular lines
	for (float j = -start; j <= start; j= j+step) {
		glVertex3f( j, 0,-start);
		glVertex3f(j, 0, start);
	}
	glEnd();
}

// callback function
void renderScene(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glTranslatef(0, Z_OFFSET, 0); // center the image
	//gluLookAt(eye_x, eye_y, eye_z, 0, 0, 0, 0.0f, 1.0f, 0.0f); //eye, center, up
	gluLookAt(getX(), getY(), getZ(), 0, 0, 0, 0.0f, 1.0f, 0.0f);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, orange);
	//glPrint(0,0,-MOTOR_CYL_HGT_S);
	gl_print_warnings(0, -MOTOR_CYL_HGT_S, -MOTOR_CYL_HGT_S *0.5);  // x-y-z
	gl_print_errors(0, -MOTOR_CYL_HGT_S, -0.5 * (MOTOR_CYL_HGT_S + MOTOR_CYL_HGT_S * 0.50));
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, ROBOT_COLOUR);
	x_y_z_draw();
	drawTable(0.10); // Draw a table with a grid

	
	/* path drawing using circular buffer */
	distance = distance_3d(tcp_center, tcp_center_previous);
	//if (fabs(tcp_center.x - tcp_center_previous.x) > 0.01 || fabs(tcp_center.y - tcp_center_previous.y) > 0.01 || fabs(tcp_center.z - tcp_center_previous.z) > 0.01) {
	//if(distance > 0.2){
	//	tcp_center_previous = tcp_center;
	//	if(path_max < PATH_MAX) path_max = i + 1;
	//	if (i >= PATH_MAX) { // reaches its maximum value
	//		i = 0; // rewinds
	//		path_max = PATH_MAX;
	//	}
	//	i++;
	//	tcp_array[i-1] = tcp_center;
	//	//dynamic_tcp_array.resize(++i);
	//	//dynamic_tcp_array[i-1] = tcp_center;
	//}

	/* Update every 10 mS*/
	if (tick_diff(stamp2) >= 10) {
		stamp2 = tick_get();
		tcp_center_previous = tcp_center;
		if (path_max < PATH_MAX) path_max = i + 1;
		if (i >= PATH_MAX) { // reaches its maximum value
			i = 0; // rewinds
			path_max = PATH_MAX;
		}
		i++;
		tcp_array[i - 1] = tcp_center;
	}
	//glLineWidth(3.0f);
	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
	//glBegin(GL_LINES);
	//glVertex3f(-TOP / 2, TOP / 2, 0); // y-z-x
	//glVertex3f(-TOP / 2, 0, 0);
	//glEnd();

	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, orange);
	/*for (int k = 0; k < path_max; k = k + 1) { // draw lines
		glBegin(GL_LINES);
		glVertex3f(-tcp_array[k].y * SCALE, tcp_array[k].z * SCALE, tcp_array[k].x * SCALE);
		if (i > 0)	glVertex3f(-tcp_array[k-1].y * SCALE, tcp_array[k-1].z * SCALE, tcp_array[k-1].x * SCALE);
		else glVertex3f(-tcp_array[path_max - 1].y * SCALE, tcp_array[path_max - 1].z * SCALE, tcp_array[path_max - 1].x * SCALE);
		glEnd();
	}*/
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, orange);
	glPointSize(3.0); // size of the tcp trail
	for (int k = 0; k < path_max; k=k+1) { // draw dots
			glBegin(GL_POINTS);
			//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cyan);
			glVertex3f(0, 0, 0);
			glVertex3f(-tcp_array[k].y * SCALE, tcp_array[k].z * SCALE, tcp_array[k].x * SCALE);
			//glVertex3f(-dynamic_tcp_array[k].y * SCALE, dynamic_tcp_array[k].z * SCALE, dynamic_tcp_array[k].x * SCALE);
			glEnd();
	}


	/* Update every 10 mS*/
	//if (tick_diff(stamp2) >= 10) {
	//		stamp2 = tick_get();
	//		glBegin(GL_POINTS);
	//		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cyan);
	//		glVertex3f(0, 0, 0);
	//		glVertex3f(-tcp_center.y * SCALE, tcp_center.z * SCALE, tcp_center.x * SCALE);
	//		glVertex3f(-dynamic_tcp_array[k].y * SCALE, dynamic_tcp_array[k].z * SCALE, dynamic_tcp_array[k].x * SCALE);
	//		glEnd();
	//}
	/* Dots to help alignment */
	//glPushMatrix();
	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, black);
	//glTranslatef(-MOTOR_CYL_RADIUS_S * 1.3, BASE_HGT_S, 0);
	//glutSolidSphere(0.05, 50, 50);
	//glPopMatrix();

	//glPushMatrix();
	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, black);
	//glTranslatef(-MOTOR_CYL_RADIUS_S * 1.3, BASE_HGT_S + HUMERUS_S, 0);
	//glutSolidSphere(0.05, 50, 50);
	//glPopMatrix();

	//glPushMatrix();
	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, black);
	//glTranslatef(-MOTOR_CYL_RADIUS_S * 1.3, BASE_HGT_S + HUMERUS_S + ULNA_S, 0);
	//glutSolidSphere(0.05, 50, 50);
	//glPopMatrix();

	//glPushMatrix();
	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, black);
	//glTranslatef(-MOTOR_CYL_RADIUS_S * 1.3, BASE_HGT_S + HUMERUS_S + ULNA_S + GRIPPER_S, 0);
	//glutSolidSphere(0.04, 50, 50);
	//glPopMatrix();

	//glPushMatrix();
	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, black);
	//glTranslatef(MOTOR_CYL_RADIUS_S / 2, BASE_HGT_S + HUMERUS_S + ULNA_S + GRIPPER_TCP_S, 0);
	//glutSolidSphere(0.04, 50, 50);
	//glPopMatrix();

		/* State machine */
	switch (state) {
	case SM_PICK:
		/* Draws vial */
		glPushMatrix();
		if(touched_f ==1 )glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green);
		else if (touched_f == 0)glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, yellow);
		glTranslatef(-line1.value[1] * SCALE, line1.value[2] * SCALE, line1.value[0] * SCALE); //y,z,x
		glRotatef(90, 1, 0, 0);
		glTranslatef(0, 0, -VIAL_HGT_S / 2); //y,z,x
		glutSolidCylinder(VIAL_RADIUS_S, VIAL_HGT_S, 20, 20);
		glPopMatrix();

		vial_center.x = line1.value[0];
		vial_center.y = (line1.value[1]);
		vial_center.z = (line1.value[2]);
		grip_to_hor = -90 + angle.value[1] + angle.value[2] + angle.value[3];
		distance = distance_3d(tcp_center, vial_center);
		if ((fabs(distance) < 0.1) && (fabs(grip_to_hor) < 5)) touched_f =1; 
		//if ((fabs(distance) < 10) && (fabs(grip_to_hor) < 25)) touched_f =1; // debug mode
		else touched_f = 0;
		if ((touched_f == 1) && ((grip == GRIP_CLOSE))) {
			state = SM_CARRY;
			touched_f = 0;
		}
		break;
	case SM_CARRY:
		/* Alignment disc */
		glPushMatrix();
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, grey);
		glTranslatef(-line2.value[1] * SCALE, line2.value[2] * SCALE, line2.value[0] * SCALE); //y,z,x
		glRotatef(90, 1, 0, 0);
		glutSolidCylinder(VIAL_RADIUS_S * 1.1, VIAL_RADIUS_S / 20, 20, 20);
		glPopMatrix();

		vial_center.x = line2.value[0];
		vial_center.y = (line2.value[1]);
		vial_center.z = (line2.value[2]);
		grip_to_hor = -90 + angle.value[1] + angle.value[2] + angle.value[3];
		distance = distance_3d(tcp_center, vial_center);
		if ((fabs(distance) < 0.1) && (fabs(grip_to_hor) < 5)) touched_f = 1;
		//if ((fabs(distance) < 5) && (fabs(grip_to_hor) < 15)) touched_f = 1; // debug mode
		else touched_f = 0;

		if ((touched_f == 1) && ((grip == GRIP_OPEN)))state = SM_DROP;
		if ((touched_f == 0) && ((grip == GRIP_OPEN))) {
			state = SM_FALL;
			fall_center= tcp_center;
			touched_f = 0;
		}
		break;
	case SM_DROP:
		/* Draws vial */
		glPushMatrix();
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green);
		glTranslatef(-line2.value[1] * SCALE, line2.value[2] * SCALE, line2.value[0] * SCALE); //y,z,x
		glRotatef(90, 1, 0, 0);
		glTranslatef(0, 0, -VIAL_HGT_S / 2); //y,z,x
		glutSolidCylinder(VIAL_RADIUS_S, VIAL_HGT_S, 20, 20);
		glPopMatrix();

		/* waits F1 to be pressed to return to SM_PICK*/
		break;

	case SM_FALL:
		/* Draws vial */
		glPushMatrix();
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, yellow);
		glTranslatef(-fall_center.y * SCALE, VIAL_RADIUS_S, fall_center.x * SCALE); //y,z,x
		glRotatef(90, 0, 1, 0);
		glutSolidCylinder(VIAL_RADIUS_S, VIAL_HGT_S, 20, 20);
		glPopMatrix();
		/* waits F1 to be pressed to return to SM_PICK*/
		break;

	//case SM_WAIT:
	//	break;
	}// end of state machine

	/* Draws pick pedestal*/
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cyan);
	glTranslatef(-line1.value[1] * SCALE, (line1.value[2] * SCALE), line1.value[0] * SCALE); //y,z,x
	glRotatef(90, 1, 0, 0);
	glTranslatef(0, 0,  (VIAL_HGT_S / 2)); //y,z,x
	glutSolidCylinder(PED_RADIUS_S, PED_HGT_S, 20, 20);
	glPopMatrix();

	/* Draws drop pedestal*/
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cyan);
	glTranslatef(-line2.value[1] * SCALE, (line2.value[2] * SCALE), line2.value[0] * SCALE); //y,z,x
	glRotatef(90, 1, 0, 0);
	glTranslatef(0, 0, (VIAL_HGT_S / 2)); //y,z,x
	glutSolidCylinder(PED_RADIUS_S, PED_HGT_S, 20, 20);
	glPopMatrix();

	/* Draws vial */
	if (state == SM_PICK) {
		//glPushMatrix();
		//if (color_f == E_GREEN)	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green);
		//else if(color_f == E_YELLOW) glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, yellow);
		//else if(color_f == E_RED) glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
		//glTranslatef(-line1.value[1] * SCALE, line1.value[2] * SCALE, line1.value[0] * SCALE); //y,z,x
		//glRotatef(90, 1, 0, 0);
		//glTranslatef(0, 0, -VIAL_HGT_S/2); //y,z,x
		//glutSolidCylinder(VIAL_RADIUS_S, VIAL_HGT_S, 20, 20);
		//glPopMatrix();
	}
	else if (state == SM_DROP) {
		//glPushMatrix();
		//if (color_f == E_GREEN)	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green);
		//else if (color_f == E_YELLOW) glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, yellow);
		//else if (color_f == E_RED) glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
		//glTranslatef(-line2.value[1] * SCALE, line2.value[2] * SCALE, line2.value[0] * SCALE); //y,z,x
		//glRotatef(90, 1, 0, 0);
		//glTranslatef(0, 0, -VIAL_HGT_S/2); //y,z,x
		//glutSolidCylinder(VIAL_RADIUS_S, VIAL_HGT_S, 20, 20);
		//glPopMatrix();
	}
	else if (state == SM_FALL) {
		//glPushMatrix();
		//if (color_f == E_GREEN)	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green);
		//else if (color_f == E_YELLOW) glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, yellow);
		//else if (color_f == E_RED) glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
		//glTranslatef(-tcp_center.y * SCALE, VIAL_HGT_S / 2, tcp_center.x * SCALE); //y,z,x
		//glRotatef(90, 1, 0, 0);
		//glTranslatef(0, 0, -VIAL_HGT_S / 2); //y,z,x
		//glutSolidCylinder(VIAL_RADIUS_S, VIAL_HGT_S, 20, 20);
		//glPopMatrix();
	}

	/* Alignment disc */
	if (state == SM_CARRY) {
		//glPushMatrix();
		//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, grey);
		//glTranslatef(-line2.value[1] * SCALE, line2.value[2] * SCALE, line2.value[0] * SCALE); //y,z,x
		//glRotatef(90, 1, 0, 0);
		//glutSolidCylinder(VIAL_RADIUS_S*1.1, VIAL_RADIUS_S / 20, 20, 20);
		//glPopMatrix();
	}

	/* Draws robots */
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, ROBOT_COLOUR);
	glRotatef(90, 1, 0, 0); //x,z,y
	glRotatef(90, 0, 0, 1); //y,z,x
	glRotatef(-base, 0, 0, 1);
	glutSolidCylinder(MOTOR_CYL_RADIUS_S, -MOTOR_CYL_HGT_S, 20, 20);  // base joint
	glRotatef(-90, 1, 0, 0); //x,y,z
	glTranslatef(0, MOTOR_CYL_HGT_S + MOTOR_CYL_RADIUS_S, 0);
	glTranslatef(0, 0, -MOTOR_CYL_HGT_S / 2);
	glutSolidCylinder(MOTOR_CYL_RADIUS_S, MOTOR_CYL_HGT_S, 20, 20);  // shoulder joint
	glRotatef(90, 1, 0, 0);
	glRotatef(90, 0, 1, 0);
	glRotatef(-shld + 90, 0, 1, 0);
	glTranslatef(0, ARM_CYL_RADIUS_S, 0);//
	glutSolidCylinder(ARM_CYL_RADIUS_S, HUMERUS_S, 20, 20); // humerus
	glTranslatef(0, 0, HUMERUS_S);
	glRotatef(-90, 1, 0, 0);
	glTranslatef(0, 0, -ARM_CYL_RADIUS_S);
	glutSolidCylinder(MOTOR_CYL_RADIUS_S, MOTOR_CYL_HGT_S, 20, 20); //elbow joint
	glRotatef(90, 1, 0, 0);
	glRotatef(-elb, 0, 1, 0);
	glTranslatef(0, MOTOR_CYL_HGT_S - ARM_CYL_RADIUS_S, 0);
	glutSolidCylinder(ARM_CYL_RADIUS_S, ULNA_S, 20, 20); // ulna
	glTranslatef(0, 0, ULNA_S);
	glRotatef(-90, 1, 0, 0);
	glTranslatef(0, 0, -ARM_CYL_RADIUS_S);
	glRotatef(-90, 0, 0, 1);
	glTranslatef(0, 0, -MOTOR_CYL_HGT_S + 2 * ARM_CYL_RADIUS_S);
	glutSolidCylinder(MOTOR_CYL_RADIUS_S, MOTOR_CYL_HGT_S, 20, 20); // gripper joint
	glRotatef(90, 1, 0, 0);
	glRotatef(90, 0, 1, 0);
	glTranslatef(0, MOTOR_CYL_HGT_S / 2, 0);
	glRotatef(-wrst, 0, 1, 0);

	/* Alignment disc */
	if (state == SM_PICK) {
		glTranslatef(0, 0, GRIPPER_TCP_S); 
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, grey);
		glRotatef(90, 0, 1, 0);
		glutSolidCylinder(VIAL_RADIUS_S*1.1, VIAL_RADIUS_S/20, 20, 20); 
		glRotatef(-90, 0, 1, 0);
		glTranslatef(0, 0, -GRIPPER_TCP_S);
	}

	/* Carried vial */
	if (state==SM_CARRY) {	
		if (touched_f == 1)glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green);
		else if (touched_f == 0)glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, yellow);
		glTranslatef(0, 0, GRIPPER_TCP_S);
		glRotatef(90, 0, 1, 0);
		glTranslatef(0, 0, -VIAL_HGT_S / 2);
		glutSolidCylinder(VIAL_RADIUS_S, VIAL_HGT_S, 20, 20);
		glTranslatef(0, 0, VIAL_HGT_S / 2);
		glRotatef(-90, 0, 1, 0);
		glTranslatef(0, 0, -GRIPPER_TCP_S);
	}

	/* Draws gripper */
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, ROBOT_COLOUR);
	if (grip == GRIP_CLOSE) {
		glTranslatef(0, -VIAL_RADIUS_S- GRIP_CYL_RADIUS_S, 0);
		glutSolidCylinder(GRIP_CYL_RADIUS_S, GRIPPER_S, 20, 20); // gripper tube
		glTranslatef(0, 2 * (VIAL_RADIUS_S+ GRIP_CYL_RADIUS_S), 0);
		glutSolidCylinder(GRIP_CYL_RADIUS_S, GRIPPER_S, 20, 20); // gripper tube
	}
	else {
		glTranslatef(0, -MOTOR_CYL_HGT_S / 2 + 2*GRIP_CYL_RADIUS_S, 0);
		glutSolidCylinder(GRIP_CYL_RADIUS_S, GRIPPER_S, 20, 20); // gripper tube
		glTranslatef(0, +MOTOR_CYL_HGT_S - 2*GRIP_CYL_RADIUS_S * 2, 0);
		glutSolidCylinder(GRIP_CYL_RADIUS_S, GRIPPER_S, 20, 20); // gripper tube
	}
	glPopMatrix();

	/* Update every 10 mS*/
	if (tick_diff(stamp) >= 10) {
		base = -pv_angle_get(0);
		shld = pv_angle_get(1);
		elb = pv_angle_get(2);
		wrst = pv_angle_get(3);
		stamp = tick_get();
		grip = gripper_get();
		//angle.value[0] = sp_angle_get(0);
		//angle.value[1] = sp_angle_get(1);
		//angle.value[2] = sp_angle_get(2);
		//angle.value[3] = sp_angle_get(3);
		angle.value[0] = pv_angle_get(0);
		angle.value[1] = pv_angle_get(1);
		angle.value[2] = pv_angle_get(2);
		angle.value[3] = pv_angle_get(3);
		tcp_center = to_cart2(angle);
		//dynamic_tcp_array.push_back(tcp_center);
	}

	glutSwapBuffers();
	//glFlush();
}

void changeSize(int w, int h) {
	float ratio = w * 1.0 / h;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);
	gluPerspective(90, ratio, 1, 100);
	glMatrixMode(GL_MODELVIEW);
}

//void pressNormalKeys(unsigned char key, int x, int y) {
//	if (key == 'w') {
//
//	}
//
//	if (key == 's') {
//
//	}
//}

void pressSpecialKey(int key, int x, int y) {
	if (key == GLUT_KEY_UP) {
		//eye_y += 0.1;
		moveUp();
	}
	if (key == GLUT_KEY_DOWN) {
		//eye_y -= 0.1;
		moveDown();
	}
	if (key == GLUT_KEY_LEFT) {
		//eye_x += 0.1;
		moveLeft();
	}
	if (key == GLUT_KEY_RIGHT) {
		//eye_x -= 0.1;
		moveRight();
	}
	if (key == GLUT_KEY_F1) {
		state = SM_PICK;
	}
	/*if (key == GLUT_KEY_F1) {
		eye_y = 0;
	}
	if (key == GLUT_KEY_F2) {
		eye_y = 3;
	}
	if (key == GLUT_KEY_F7) {
		eye_y -= 0.1;
	}
	if (key == GLUT_KEY_F8) {
		eye_y += 0.1;
	}*/
}


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
// Performs application specific initialization.  It defines lighting
// parameters for light source GL_LIGHT0: black for ambient, yellow for
// diffuse, white for specular, and makes it a directional source
// shining along <-1, -1, -1>.  It also sets a couple material properties
// to make cyan colored objects with a fairly low shininess value.  Lighting
// and depth buffer hidden surface removal are enabled here.
void init() {
	glClearColor(1, 1, 1, 1.0); // white background
	GLfloat direction[] = { 1.0, 1.0, 1.0, 0.0 }; // infinity
	//GLfloat direction[] = { 1.0, 1.0, 0.0, 0.0 }; // xy infinity
	//GLfloat direction[] = { 1.0, 1.0, 1.0, 1.0 }; //at a point in space
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red); // robot's color
	glMaterialfv(GL_FRONT, GL_SPECULAR, white); 
	glMaterialf(GL_FRONT, GL_SHININESS, 30);
	//glMaterialf(GL_FRONT, GL_SHININESS, 90);

	//glLightfv(GL_LIGHT0, GL_AMBIENT, black);
	//glLightfv(GL_LIGHT0, GL_AMBIENT, white);
	//glLightfv(GL_LIGHT0, GL_DIFFUSE, yellow);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, white);
	glLightfv(GL_LIGHT0, GL_POSITION, direction);

	glEnable(GL_LIGHTING);                // so the renderer considers light
	glEnable(GL_LIGHT0);                  // turn LIGHT0 on
	glEnable(GL_DEPTH_TEST);              // so the renderer considers depth
	
}

// Application-specific initialization: Set up global lighting parameters
// and create display lists.
void init2() {
	glClearColor(1, 1, 1, 1.0); // white background
	glEnable(GL_DEPTH_TEST);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
	glLightfv(GL_LIGHT0, GL_SPECULAR, white);
	glMaterialfv(GL_FRONT, GL_SPECULAR, white);
	glMaterialf(GL_FRONT, GL_SHININESS, 30);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	//glEnable(GL_COLOR_MATERIAL);
	//glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
}

void init3() {
		//GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 }; // infinity
		GLfloat light_position[] = { 1.0, 1.0, 1.0, 1.0 }; //at a point in space
		//GLfloat light_position[] = { 0.0, TOP, 0.0 , 1.0 }; //yzx
		//GLfloat light_position[] = { 1.0, 0.0, 1.0, 1.0 }; //yzx
		//GLfloat light_position[] = { 1.0, 0.0, 1.0, 1.0 }; //yzx
		//GLfloat light_position[] = { 0.0, 0.0, 1.0, 1.0 }; //yzx
		//glClearColor(1, 1, 1, 1.0); // white background
		//glClearColor(0.68f, 0.85f, 0.90f, 1.0f );  // pale blue
		glClearColor( 0.53f, 0.81f, 0.92f, 1.0f ); // sky blue
		//glClearColor( 0.00f, 0.50f, 0.50f, 1.0f ); // marine blue
		//glClearColor(0.8f, 0.8f, 0.8f, 1.0); // grey background
		glLightfv(GL_LIGHT0, GL_POSITION, light_position);
		//glLightfv(GL_LIGHT0, GL_POSITION, white);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		//glEnable(GL_COLOR_MATERIAL);
		//glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
}


void MyMouse(int button, int state, int x, int y)
{
	//y,z,x
	//eye_z =(y-442)/50;
	//eye_x = (x-325)/50;
}

/*
	TaskOpenGL
	Animation
*/
static void* pTaskOpenGl(void* ptr) {
	//make it cancellable
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
	int argc = 0;
	char** argv = 0;
	// Reads line 1
	file_read_("mover4/sol_mover4/vial.txt", &line1);
	// Reads line 2
	file_read_("mover4/sol_mover4/vial.txt", &line2);

	tick_init();
	stamp = tick_get();

	/* init variables */
	base = -pv_angle_get(0);
	shld = pv_angle_get(1);
	elb = pv_angle_get(2);
	wrst = pv_angle_get(3);
	stamp = tick_get();
	grip = gripper_get();
	angle.value[0] = sp_angle_get(0);
	angle.value[1] = sp_angle_get(1);
	angle.value[2] = sp_angle_get(2);
	angle.value[3] = sp_angle_get(3);
	tcp_center = to_cart2(angle);


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
	glutSpecialFunc(pressSpecialKey);
	//glutMouseFunc(MyMouse);
	//init(); // for lighting purpose - glColor() is disabled automatically
	init3();
	glutMainLoop();
	return 0;
}


static int file_read_(const char* name, data_t* data) {
	static char line[100]; // Assuming a maximum line length of 100 characters
	int i = 0;
	static FILE* file_r = NULL; // File pointer

	if (file_r == NULL) {
		//open = 1; // Opens the file only the first time
		file_r = fopen(name, "r");
		if (file_r == NULL) {
			perror("Error opening the file.");
			return 1;
		}
	}
	//while (file_read("mover4/sol_mover4/vial.txt", &data) == 0);
	char* token;
	char* endptr;
	if (fgets(line, sizeof(line), file_r)) { // Reads a complete line
		token = strtok(line, " "); // Space-separated values
		if (token == NULL) {
			fseek(file_r, 0, SEEK_SET);
		}
		while (token != NULL) {
			double x = strtof(token, &endptr);
			data->value[i++] = x;
			token = strtok(NULL, " ");
		}
	}
}

static double to_radians2(double degrees) {
	return (degrees * 3.1416) / 180;
}

/* Converts from angles to cartesian values */
static Point3D to_cart2(data_f angle) {
	Point3D cart;
	double r;
	r = HUMERUS * cos(to_radians2(90 - angle.value[1])) + ULNA * cos(to_radians2(90 - angle.value[1] - angle.value[2])) + GRIPPER * cos(to_radians2(90 - angle.value[1] - angle.value[2] - angle.value[3]));
	// z
	cart.z = BASE_HGT + HUMERUS * sin(to_radians2(90 - angle.value[1])) + ULNA * sin(to_radians2(90 - angle.value[1] - angle.value[2])) + GRIPPER * sin(to_radians2(90 - angle.value[1] - angle.value[2] - angle.value[3]));
	// y
	cart.y = r * sin(to_radians2(angle.value[0]));
	// x
	cart.x = r * cos(to_radians2(angle.value[0]));
	return cart;
}

/* Draws the z-y-z axis*/
static void x_y_z_draw(void) {
	//glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, black);
	glLineWidth(1);
	//z axis
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, TOP + GRIPPER_S*0.5, 0);
	glEnd();
	glPushMatrix();
	glTranslatef(0, TOP + GRIPPER_S * 0.5, 0);// y,z,x
	glRotatef(-90, 1, 0, 0);
	glutSolidCone(TOP * 0.01, TOP * 0.02, 15, 15);
	glPopMatrix();
	//glRotatef(90, 1, 0, 0);
	//glTranslatef(0, -TOP + GRIPPER_S * 0.5, 0);;// y,z,x
	// z letter
	glRasterPos3f(-TOP / 40, TOP + GRIPPER_S*0.6,0);// position
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'Z');

	//x axis
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, TOP * 0.65);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, -TOP * 0.65);
	glEnd();
	glPushMatrix();
	glTranslatef(0,0, TOP * 0.65);// y,z,x
	glutSolidCone(TOP*0.01, TOP * 0.02, 15,15);
	glPopMatrix();
	//glTranslatef(0, 0, -TOP * 0.65);// y,z,x

	// x letter
	glRasterPos3f(-TOP / 40, TOP / 40, TOP * 0.2);// position
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'X');

	//y axis
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(-TOP * 0.65, 0, 0);
	glEnd();

	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(TOP * 0.65, 0, 0);
	glEnd();

	glPushMatrix();
	glTranslatef(-TOP * 0.65,0,0);// y,z,x
	glRotatef(-90, 0, 1, 0);
	glutSolidCone(TOP * 0.01, TOP * 0.02, 15, 15);
	glPopMatrix();

	// y letter
	glRasterPos3f(-TOP * 0.2, TOP / 40, -TOP / 40);// position
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, 'Y');
	
}

// Function to calculate the distance between two 3D points
static double distance_3d(Point3D point1, Point3D point2) {
	double dx = point2.x - point1.x;
	double dy = point2.y - point1.y;
	double dz = point2.z - point1.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}


//void glPrint(double x, double y, double z) {
//	int i = 0;
//	char text[] = "this is a test";
//	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
//	glPushMatrix();
//	glRasterPos3f(y, z, x);// position
//	while (text[i] != '\0') {
//		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, text[i++]);
//	}
//	glPopMatrix();
//}