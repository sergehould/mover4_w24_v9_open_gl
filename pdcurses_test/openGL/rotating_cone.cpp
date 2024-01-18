/*////////////////////////////////////////////////

	SH	2023


	Note: to set platform, properties->General-> platform Toolset -> select Visual Studio 2019 (v142)

////////////////////////////////////////////////*/


#include "freeglut/include/GL/glut.h"
#include "freeglut/include/GL/freeglut.h"
#include <stdio.h>
#include <windows.h>
#include <string.h>
#include <math.h>
#include "../../common/tick.h"

const float PI = 3.141593;

LARGE_INTEGER li;

#define RADIUS1	2.0
#define RADIUS2	1.0
long stamp;

// callback function
void renderScene(void)
{
	static float angle = 0;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1, 1, 1.0f, 0);
	glLoadIdentity();
	//glTranslatef(0, 0, -1);
	glTranslatef(0, 0, -5);
	gluLookAt(0, 0, 1, 0, 0, 0, 0.0f, 1.0f, 0.0f); //eye, center, up
	glColor3f(0, 0, 1);
	//gluLookAt(0, 0.0f, 1.0f,
	//	0.0f, 0.0f, 0.0f,
	//	0.0f, 1.0f, 0.0f);

	//glRotatef(angle, 0.0f, 1.0f, 0.0f);
	glutWireSphere(RADIUS1, 100, 20);
	glTranslatef((RADIUS1 + +RADIUS2) * sinf(angle), (RADIUS1 + +RADIUS2) * cosf(angle), 0);
	glColor3f(1, 0, 0);
	glRotatef(angle * 4, 0, 0, 1);
	glutWireSphere(RADIUS2, 20, 8);
	if (tick_diff(stamp) >= 10) {
		stamp = tick_get();
		angle += 2 * PI / 100; // 2PI rad/sec
		//angle += 0.0628;
	}
	//angle += 0.1f;
	glutSwapBuffers();

}

void changeSize(int w, int h)
{

	float ratio = w * 1.0 / h;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);
	gluPerspective(90, ratio, 1, 100);
	glMatrixMode(GL_MODELVIEW);
}

void pressNormalKeys(unsigned char key, int x, int y) {
	if (key == 'q') {
		//angle += 0.1f;
	}

	if (key == 'a') {
		//angle -= 0.1f;
	}
}



int main(int argc, char** argv) {
	tick_init();
	stamp = tick_get();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(10, 10);
	glutInitWindowSize(640, 640);
	glutCreateWindow("Hello World");
	glutDisplayFunc(renderScene);
	glEnable(GL_DEPTH_TEST); // to avoid back get drawn to the display over the top of the model at the front
	glutIdleFunc(renderScene);
	glutReshapeFunc(changeSize);
	glutKeyboardFunc(pressNormalKeys);
	glutMainLoop();
}

