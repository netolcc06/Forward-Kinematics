#include <cstdio>  
#include <cstdlib>  
#include "ForwardKinematic.h"  

// OpenGL call back functions' declarations
void display();
void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);

//OpenGL initialize operations
void OpenGLinit(int argc, char* argv[]);

//create an instance of FK class
ForwardKinematic FK_Model;

//Main enntry
int main(int argc, char *argv[])
{
	//OpenGL initilize operations
	OpenGLinit(argc, argv);

	//Load bvh file and reconstruct the skeleton
	FK_Model.load_BVH("../data/running.bvh");

	//Print the loaded human skeleton
	FK_Model.print();

	//While loop
	while (true)
	{
 		//response to the keyboard event: presing "w", load next frame's data to 'pFrame'
		glutMainLoopEvent(); 

		//recalculate the skeleton joints' positions with new frame's data
		FK_Model.calculateJointPos(); 

		glutPostRedisplay(); //re-display the new skeleton
	}

	return 0;
}


//OpenGL initilize operations
void OpenGLinit(int argc, char* argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowPosition(300, 75);
	glutInitWindowSize(800, 600);
	glutCreateWindow("Forward Kinematics");
	glShadeModel(GL_SMOOTH);
	glClearColor(0.0f, 1.0f, 1.0f, 0.0f); //initial background
	glEnable(GL_DEPTH_TEST);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutDisplayFunc(display);
}

//OpenGL call back function: display the skeleton for current frame
void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(200.0f*cos(10.0f), 100.0f, 200.0f*sin(10.0f), 0, 20, 0, 0, 1, 0);

	//draw floor
	glColor3f(1.0f, 1.0f, 0.0f);
	glBegin(GL_POLYGON);
	glVertex3f(500.0f, -100.0f, -500.0f);
	glVertex3f(500.0f, -100.0f, 500.0f);
	glVertex3f(-500.0f, -100.0f, 500.0f);
	glVertex3f(-500.0f, -100.0f, -500.0f);
	glEnd();

	//model.calculateJointPos();
	FK_Model.draw();
	glutSwapBuffers();
}

//OpenGL call back fucntion: reshape display window function
void reshape(int w, int h)
{
	glViewport(0, 0, GLsizei(w), GLsizei(h));
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, (GLdouble)w / (GLdouble)h, 1.0f, 1000.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(200.0f*cos(10.0f), 0, 200.0f*sin(10.0f), 0, 0, 0, 0, 1, 0);
}

//OpenGL call back function: kepboard responsing functiion: once w is pressed on keyboard, one more frame data is loaded
void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(0);
	case 'w':
	case 'W':
		FK_Model.addFrame();
	}
	//glutPostRedisplay();

}