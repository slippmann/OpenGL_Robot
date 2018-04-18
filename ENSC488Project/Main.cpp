#define GL_GLEXT_PROTOTYPES

#include <GL/glut.h>
#include <math.h>
#include "Point2D.h"
#include "Point3D.h"
#include "Colour.h"
#include <ctime>
#include <iostream>

// Comment out to disable AntTweakBar
#include <AntTweakBar.h>

// Uncomment to enable debugging messages in terminal
//#define DEBUG

#define PI				3.14159265358979323846

#define WINDOW_HEIGHT	900
#define WINDOW_WIDTH	WINDOW_HEIGHT

float zoom = 1.0;

enum direction {
	N = 90, 
	NE = 45,
	E = 0,
	SE = -45,
	S = -90,
	SW = -135,
	W = 180,
	NW = 135
};

enum keys {
	NONE = 0,
	UP = 0x01,
	DOWN = 0x10,
	LEFT = 0x02,
	RIGHT = 0x20
};

//==============GL Parameters============

unsigned char currentKeys = (unsigned char)NONE;

// Rotate X veiw
int rX = 75;
// Rotate Z veiw
int rZ = 15;

// =============Robot Parameters===========

Point3D newLocation;
Point3D viaLocation;
Point3D Location = Point3D(0, 0, 0);
float dir = 0;

float speed;
float trajTime = 1;
float elapsedTime = 0;
bool leftTurn = true;

Point3D PrevEndPoint;
Point3D EndPoint = Point3D(50, 0, -50);
float t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0;

// DH parameters (based on PUMA robot from textbook)
float a2 = 50.0, a3 = 0.0, d3 = 0.0, d4 = 50.0;

// Rotation parameters
float PrevRotZ1, PrevRotY, PrevRotZ2;
float RotZ1, RotY, RotZ2;

// Rotation matrix
float r11 = 1, r12 = 0, r13 = 0, \
r21 = 0, r22 = 1, r23 = 0, \
r31 = 0, r32 = 0, r33 = 1;

// ===========Function Declarations===========

void drawOrigin()
{
	// make sure origin is visible
	glLineWidth(5);
	glColor3ub(255, 0, 0);

	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(50, 0, 0);
	glEnd();

	glColor3ub(0, 255, 0);

	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 50, 0);
	glEnd();

	glColor3ub(0, 0, 255);

	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 50);
	glEnd();
}

void drawRectangle(Point3D start, Point3D end, float thickness, Colour c)
{
	double length = sqrt(pow((end.x - start.x), 2) + pow((end.y - start.y), 2) + pow((end.z - start.z), 2));
	double xylength = sqrt(pow((end.x - start.x), 2) + pow((end.y - start.y), 2));
	double zAngle = atan2((end.y - start.y), (end.x - start.x)) * (180.0 / PI);
	double yAngle = atan2((end.z - start.z), xylength) * (180.0 / PI);

	glPushMatrix();

	// translate and rotate the scaled rectangular prism
	glTranslatef(start.x, start.y, start.z);
	glRotatef(zAngle, 0, 0, 1);
	glRotatef(-yAngle, 0, 1, 0);

	glPushMatrix();

	// scale the retangular prism first
	glScalef(length, thickness, thickness);

	glColor3ub(c.r, c.g, c.b);

	// Right
	glBegin(GL_QUADS);
	glVertex3f(1, -0.5, -0.5);
	glVertex3f(1, 0.5, -0.5);
	glVertex3f(1, 0.5, 0.5);
	glVertex3f(1, -0.5, 0.5);
	glEnd();

	glColor3ub(c.r / 4, c.g / 4, c.b / 4);

	// Left
	glBegin(GL_QUADS);
	glVertex3f(0, -0.5, -0.5);
	glVertex3f(0, 0.5, -0.5);
	glVertex3f(0, 0.5, 0.5);
	glVertex3f(0, -0.5, 0.5);
	glEnd();

	glColor3ub(c.r * 3 / 4, c.g * 3 / 4, c.b * 3 / 4);

	// Front
	glBegin(GL_QUADS);
	glVertex3f(1, 0.5, 0.5);
	glVertex3f(1, 0.5, -0.5);
	glVertex3f(0, 0.5, -0.5);
	glVertex3f(0, 0.5, 0.5);
	glEnd();

	// Back
	glBegin(GL_QUADS);
	glVertex3f(1, -0.5, -0.5);
	glVertex3f(1, -0.5, 0.5);
	glVertex3f(0, -0.5, 0.5);
	glVertex3f(0, -0.5, -0.5);
	glEnd();

	glColor3ub(c.r / 2, c.g / 2, c.b / 2);

	// Bottom
	glBegin(GL_QUADS);
	glVertex3f(1, 0.5, -0.5);
	glVertex3f(1, -0.5, -0.5);
	glVertex3f(0, -0.5, -0.5);
	glVertex3f(0, 0.5, -0.5);
	glEnd();

	// Top
	glBegin(GL_QUADS);
	glVertex3f(1, -0.5, 0.5);
	glVertex3f(1, 0.5, 0.5);
	glVertex3f(0, 0.5, 0.5);
	glVertex3f(0, -0.5, 0.5);
	glEnd();

	glPopMatrix();

	glPopMatrix();
}

void drawCylinder(Point3D start, Point3D end, float radius, Colour c)
{
	const int NUM_SEGMENTS = 100;
	double length = sqrt(pow((end.x - start.x), 2) + pow((end.y - start.y), 2) + pow((end.z - start.z), 2));
	double xylength = sqrt(pow((end.x - start.x), 2) + pow((end.y - start.y), 2));
	double zAngle = atan2((end.y - start.y), (end.x - start.x)) * (180.0 / PI);
	double yAngle = atan2((end.z - start.z), xylength) * (180.0 / PI);
	
	double twoPi = 2.0*PI;

	glPushMatrix();

	// translate and rotate the scaled rectangular prism
	glTranslatef(start.x, start.y, start.z);
	glRotatef(zAngle, 0, 0, 1);
	glRotatef(-yAngle, 0, 1, 0);

	glPushMatrix();

	// scale the cylinder first
	glScalef(length, radius, radius);

	glColor3ub(c.r, c.g, c.b);

	glBegin(GL_TRIANGLE_FAN);
	glVertex3f(1, 0, 0);
	for (int i = 0; i < NUM_SEGMENTS; i++)
	{
		float theta = ((float)i)*twoPi / (float)NUM_SEGMENTS;
		float nextTheta = ((float)i + 1)*twoPi / (float)NUM_SEGMENTS;
 
		glVertex3f(1, cos(theta), sin(theta));
		glVertex3f(1, cos(nextTheta), sin(nextTheta));
	}
	glEnd();

	glColor3ub(c.r / 4, c.g / 4, c.b / 4);

	glBegin(GL_TRIANGLE_FAN);
	glVertex3f(0, 0, 0);
	for (int i = 0; i < NUM_SEGMENTS; i++)
	{
		float theta = ((float)i)*twoPi / (float)NUM_SEGMENTS;
		float nextTheta = ((float)i + 1)*twoPi / (float)NUM_SEGMENTS;

		glVertex3f(0, cos(theta), sin(theta));
		glVertex3f(0, cos(nextTheta), sin(nextTheta));
	}
	glEnd();

	glColor3ub(c.r * 3 / 4, c.g * 3 / 4, c.b * 3 / 4);

	glBegin(GL_TRIANGLE_STRIP);
	for (int i = 0; i < NUM_SEGMENTS; i++)
	{
		float theta = ((float)i)*twoPi / (float)NUM_SEGMENTS;
		float nextTheta = ((float)i + 1)*twoPi / (float)NUM_SEGMENTS;

		glVertex3f(0, cos(theta), sin(theta));
		glVertex3f(0, cos(nextTheta), sin(nextTheta));
		glVertex3f(1, cos(theta), sin(theta));
		glVertex3f(1, cos(nextTheta), sin(nextTheta));
	}
	glEnd();

	glPopMatrix();
	glPopMatrix();
}

void calculateIK(Point3D pt)
{
	float a, b; // temporary values
	float theta1, theta2, theta3, theta4, theta5, theta6; // calculated angles

	float sx = sin(-RotZ1 * (PI / 180.0));
	float cx = cos(-RotZ1 * (PI / 180.0));
	float sy = sin(-RotY * (PI / 180.0));
	float cy = cos(-RotY * (PI / 180.0));
	float sz = sin(-RotZ2 * (PI / 180.0));
	float cz = cos(-RotZ2 * (PI / 180.0));

	// determine rotation matrix entries
	r11 = cx*cy*cz - sx*sz;
	r12 = -cz*sx - cx*cy*sz;
	r13 = cx*sy;
	r21 = cx*sz + cy*cz*sx;
	r22 = cx*cz - cy*sx*sz;
	r23 = sx*sy;
	r31 = -cz*sy;
	r32 = sy*sz;
	r33 = cy;

	theta1 = atan2(pt.y, pt.x) - atan2(d3, sqrtf(pow(pt.x, 2) + pow(pt.y, 2) - pow(d3, 2)));

	if (theta1 * (180.0 / PI) > 180 || theta1 * (180.0 / PI) < -180)
	{
		theta1 = atan2(pt.y, pt.x) - atan2(d3, -sqrtf(pow(pt.x, 2) + pow(pt.y, 2) - pow(d3, 2)));
	}

	float c1 = cos(theta1);
	float s1 = sin(theta1);

	float K = (pow(pt.x, 2) + pow(pt.y, 2) + pow(pt.z, 2) - pow(a2, 2) - pow(a3, 2) - pow(d3, 2) - pow(d4, 2)) / (2 * a2);

	theta3 = atan2(a3, d4) - atan2(K, sqrtf(pow(a3, 2) + pow(d4, 2) - pow(K, 2)));


	if (theta3 * (180.0 / PI) > 180 || theta3 * (180.0 / PI) < -180)
	{
		theta3 = atan2(a3, d4) - atan2(K, -sqrtf(pow(a3, 2) + pow(d4, 2) - pow(K, 2)));
	}

	float c3 = cos(theta3);
	float s3 = sin(theta3);

	a = ((-a3 - (a2 * c3)) * pt.z) + (((c1 * pt.x) + (s1 * pt.y)) * ((a2 * s3) - d4));
	b = (((a2 * s3) - d4) * pt.z) + ((a3 + (a2 * c3)) * ((c1 * pt.x) + (s1 * pt.y)));

	float theta23 = atan2(a, b);

	float c23 = cos(theta23);
	float s23 = sin(theta23);

	theta2 = theta23 - theta3;

	a = ((-r13 * s1) + (r23 * c1));
	b = ((-r13 * c1 * c23) - (r23 * s1 * c23) + (r33 * s23));

	if ((a < 0.1 && a > -0.1) && (b < 0.1 && b > -0.1))
	{
		theta4 = t4 * (PI / 180.0);
	}
	else
	{
		theta4 = atan2(a, b);
	}

	float c4 = cos(theta4);
	float s4 = sin(theta4);

	float s5 = (r13 * ((c1 * c23 * c4) + (s1 * s4))) + (r23 * ((s1 * c23 * c4) - (c1 * s4))) - (r33 * (s23 * c4));
	float c5 = -(r13 * (-c1 * s23)) - (r23 * (-s1 * s23)) - (r33 * (-c23));

	theta5 = atan2(s5, c5);

	float s6 = (-r11 * ((c1 * c23 * s4) - (s1 * c4))) - (r21 * ((s1 * c23 * s4) + (c1 * c4))) + (r31 * (s23 * s4));
	float c6 = (r11 * ((((c1 * c23 * c4) + (s1 * s4)) * c5) - (c1 * s23 * s5))) + (r21 * ((((s1 * c23 * c4) - (c1 * s4)) * c5) - (s1 * s23 * s5))) - (r31 * ((s23 * c4 * c5) + (c23 * s5)));

	theta6 = atan2(s6, c6);

	t1 = theta1 * (180.0 / PI);
	t2 = -theta2 * (180.0 / PI);
	t3 = -theta3 * (180.0 / PI);
	t4 = theta4 * (180.0 / PI);
	t5 = -theta5 * (180.0 / PI);
	t6 = theta6 * (180.0 / PI);
}

void DrawPlatform()
{
	// Body
	drawCylinder(Point3D(0, 0, 10), Point3D(0, 0, 20), 50, Colour(255, 255, 255));
	// Right Wheel
	drawCylinder(Point3D(0, 20, 10), Point3D(0, 40, 10), 10, Colour(125, 0, 0));
	// Left Wheel
	drawCylinder(Point3D(0, -20, 10), Point3D(0, -40, 10), 10, Colour(125, 0, 0));
}

void DrawArm(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6)
{
	glMatrixMode(GL_MODELVIEW);
	Colour cBase = Colour(192, 192, 192);
	Colour cLink1 = Colour(128, 128, 128);
	Colour cLink2 = Colour(192, 192, 192);
	Colour cLink3 = Colour(128, 128, 128);

	Point3D base = Point3D(0, 0, 20);
	Point3D j1 = Point3D(0, 0, 40);
	Point3D j2 = Point3D(j1, 0, 0, 50);
	Point3D j3 = Point3D(j2, a2, 0, d3);
	Point3D j4 = Point3D(j3, a3, 0, -d4);

	drawRectangle(base, j1, 25, cBase);

	glPushMatrix();

	glRotatef(theta1, 0, 0, 1);

	drawRectangle(j1, j2, 16, cBase);

	glPushMatrix();

	glTranslatef(j2.x, j2.y, j2.z);
	glRotatef(theta2, 0, -1, 0);

	drawRectangle(Point3D(-5, 10, 0), Point3D(a2 + 5, 10, 0), 10, cLink1);

	glPushMatrix();

	glTranslatef((j3.x - j2.x), (j3.y - j2.y), (j3.z - j2.z));
	glRotatef(theta3, 0, -1, 0);

	drawRectangle(Point3D(0, 0, 5), Point3D(0, 0, -d4 - 5), 10, cLink2);

	glPushMatrix();

	glTranslatef((j4.x - j3.x), (j4.y - j3.y), (j4.z - j3.z));
	glRotatef(theta4, 0, 0, -1);
	glRotatef(theta5, 0, -1, 0);
	glRotatef(theta6, 0, 0, -1);

	drawRectangle(Point3D(0, 0, 5), Point3D(0, 0, -20), 8, cLink3);

	glPopMatrix();
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
}

void DrawRobot()
{
	DrawPlatform();

	if (EndPoint != PrevEndPoint ||
		RotZ1 != PrevRotZ1 ||
		RotY != PrevRotY ||
		RotZ2 != PrevRotZ2)
	{
		PrevRotZ1 = RotZ1;
		PrevRotY = RotY;
		PrevRotZ2 = RotZ2;
		PrevEndPoint = EndPoint;

		calculateIK(EndPoint);
	}

	DrawArm(t1, t2, t3, t4, t5, t6);
}

void Move()
{
	const int stepSize = 1;

	if (currentKeys & UP)
	{
		Location.y += stepSize;

		if (currentKeys & RIGHT)
		{
			Location.x += stepSize;
			dir = (float)NE;
		}
		else if (currentKeys & LEFT)
		{
			Location.x -= stepSize;
			dir = (float)NW;
		}
		else
		{
			dir = (float)N;
		}

		return;
	}

	if (currentKeys & DOWN)
	{
		Location.y -= stepSize;

		if (currentKeys & RIGHT)
		{
			Location.x += stepSize;
			dir = (float)SE;
		}
		else if (currentKeys & LEFT)
		{
			Location.x -= stepSize;
			dir = (float)SW;
		}
		else
		{
			dir = (float)S;
		}

		return;
	}

	if (currentKeys & RIGHT)
	{
		Location.x += stepSize;
		dir = (float)E;
		return;
	}
	
	if (currentKeys & LEFT)
	{
		Location.x -= stepSize;
		dir = (float)W;
		return;
	}
}

void Draw()
{
	clock_t loopBegin = clock();
	clock_t loopEnd;

	// Set Background Color
	glClearColor(0.4, 0.4, 0.4, 1.0);
	// Clear screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();

	// Rotate when user changes rX and rY
	glRotatef(rX, 1.0, 0.0, 0.0);
	glRotatef(rZ, 0.0, 0.0, 1.0);
	glScalef(zoom, zoom, zoom);

	glOrtho(-WINDOW_WIDTH, WINDOW_WIDTH, -WINDOW_WIDTH, WINDOW_WIDTH, -WINDOW_WIDTH, WINDOW_WIDTH);

	drawOrigin();

	//floor
	drawRectangle(Point3D(0, 0, -5), Point3D(0, 0, 0), 1000, Colour(0, 125, 0));

	Move();

	if (isTrajSet)
	{
		calculatePosition(elapsedTime);
	}

	glPushMatrix();
	glTranslatef(Location.x, Location.y, 0);

	if (dir > 0)
	{
		glTranslatef(0, 10, 0);
		glRotatef(dir, 0, 0, 1);
		glTranslatef(0, -10, 0);
	}
	else
	{
		glTranslatef(0, -10, 0);
		glRotatef(dir, 0, 0, 1);
		glTranslatef(0, 10, 0);
	}

	DrawRobot();
	glPopMatrix();

	glFlush();

#ifdef TW_INCLUDED
	// Draw tweak bars
	TwDraw();
#endif

	glutSwapBuffers();

	// Request display update
	glutPostRedisplay();

	loopEnd = clock();
	
	if (isTrajSet)
	{
		elapsedTime += double(loopEnd - loopBegin) / CLOCKS_PER_SEC;
	}
}

void directionHandler(int key, int x, int y)
{
	if (key == GLUT_KEY_RIGHT)
	{
		rZ += -15;
	}
	else if (key == GLUT_KEY_LEFT)
	{
		rZ += 15;
	}

	if (key == GLUT_KEY_DOWN)
	{
		rX += -15;
	}
	else if (key == GLUT_KEY_UP)
	{
		rX += 15;
	}

	// Request display update
	glutPostRedisplay();
}

void keyDownHandler(unsigned char key, int x, int y)
{
	// FORWARD/REVERSE
	if (key == 'w')
	{
		currentKeys |= (unsigned char)UP;
	}
	
	if (key == 's')
	{
		currentKeys |= (unsigned char)DOWN;
	}

	// LEFT/RIGHT
	if (key == 'd')
	{
		currentKeys |= (unsigned char)RIGHT;
	}
	
	if (key == 'a')
	{
		currentKeys |= (unsigned char)LEFT;
	}

#ifdef TW_INCLUDED
	TwEventKeyboardGLUT(key, x, y);
#endif
}

void keyUpHandler(unsigned char key, int x, int y)
{
	// FORWARD/REVERSE
	if (key == 'w')
	{
		currentKeys &= ~(unsigned char)UP;
	}
	
	if (key == 's')
	{
		currentKeys &= ~(unsigned char)DOWN;
	}

	// LEFT/RIGHT
	if (key == 'd')
	{
		currentKeys &= ~(unsigned char)RIGHT;
	}
	
	if (key == 'a')
	{
		currentKeys &= ~(unsigned char)LEFT;
	}
}

#ifdef TW_INCLUDED
void TW_CALL button_callback(void * clientData)
{
	calculateFullTrajectory(newLocation, viaLocation, speed, trajTime);
}

void initTweak()
{
	TwBar *bar;         // Pointer to a tweak bar

						// Initialize AntTweakBar
	TwInit(TW_OPENGL, NULL);

	// Set GLUT event callbacks
	// - Directly redirect GLUT mouse button events to AntTweakBar
	glutMouseFunc((GLUTmousebuttonfun)TwEventMouseButtonGLUT);
	// - Directly redirect GLUT mouse motion events to AntTweakBar
	glutMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
	// - Directly redirect GLUT mouse "passive" motion events to AntTweakBar (same as MouseMotion)
	glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
	// - Send 'glutGetModifers' function pointer to AntTweakBar;
	//   required because the GLUT key event functions do not report key modifiers states.
	TwGLUTModifiersFunc(glutGetModifiers);

	// Create a tweak bar
	bar = TwNewBar("Settings");
	TwDefine(" GLOBAL help='Manipulate the robot'"); // Message added to the help bar.
	TwDefine(" Settings size='200 200' color='192 192 192' "); // change default tweak bar size and color

	//===========Forward Kin==============
	TwAddVarRW(bar, "Theta 1:", TW_TYPE_FLOAT, &t1,
		" min=-180 max=180 step=10 group='Forward Kinematics'");
	TwAddVarRW(bar, "Theta 2:", TW_TYPE_FLOAT, &t2,
		" min=-180 max=180 step=10 group='Forward Kinematics'");
	TwAddVarRW(bar, "Theta 3:", TW_TYPE_FLOAT, &t3,
		" min=-180 max=180 step=10 group='Forward Kinematics'");
	TwAddVarRW(bar, "Theta 4:", TW_TYPE_FLOAT, &t4,
		" min=-180 max=180 step=10 group='Forward Kinematics'");
	TwAddVarRW(bar, "Theta 5:", TW_TYPE_FLOAT, &t5,
		" min=-180 max=180 step=10 group='Forward Kinematics'");
	TwAddVarRW(bar, "Theta 6:", TW_TYPE_FLOAT, &t6,
		" min=-180 max=180 step=10 group='Forward Kinematics'");

	TwDefine(" Settings/'Forward Kinematics' opened=false ");

	//===========Inverse Kin==============
	TwAddVarRW(bar, "X:", TW_TYPE_FLOAT, &EndPoint.x,
		" min=-100 max=100 step=10 group='Inverse Kinematics'");
	TwAddVarRW(bar, "Y:", TW_TYPE_FLOAT, &EndPoint.y,
		" min=-100 max=100 step=10 group='Inverse Kinematics'");
	TwAddVarRW(bar, "Z:", TW_TYPE_FLOAT, &EndPoint.z,
		" min=-100 max=100 step=10 group='Inverse Kinematics'");

	TwAddSeparator(bar, "separator1", "group='Inverse Kinematics'");

	TwAddVarRW(bar, "Rot Z1:", TW_TYPE_FLOAT, &RotZ1,
		" min=-180 max=180 step=10 group='Inverse Kinematics'");
	TwAddVarRW(bar, "Rot Y:", TW_TYPE_FLOAT, &RotY,
		" min=-180 max=180 step=10 group='Inverse Kinematics'");
	TwAddVarRW(bar, "Rot Z2:", TW_TYPE_FLOAT, &RotZ2,
		" min=-180 max=180 step=10 group='Inverse Kinematics'");

	TwDefine(" Settings/'Inverse Kinematics' opened=false ");

	//===========Trajectory==============
	TwAddVarRW(bar, "End X Pos:", TW_TYPE_FLOAT, &newLocation.x,
		" min=-400 max=400 step=10 group=Trajectory");
	TwAddVarRW(bar, "End Y Pos:", TW_TYPE_FLOAT, &newLocation.y,
		" min=-400 max=400 step=10 group=Trajectory");
	TwAddVarRW(bar, "End Z Angle:", TW_TYPE_FLOAT, &newLocation.z,
		" min=-180 max=180 step=5 group=Trajectory");

	TwAddSeparator(bar, "separator4", "group=Trajectory");

	TwAddVarRW(bar, "Via X Pos:", TW_TYPE_FLOAT, &viaLocation.x,
		" min=-400 max=400 step=10 group=Trajectory");
	TwAddVarRW(bar, "Via Y Pos:", TW_TYPE_FLOAT, &viaLocation.y,
		" min=-400 max=400 step=10 group=Trajectory");
	TwAddVarRW(bar, "Via Z Angle:", TW_TYPE_FLOAT, &viaLocation.z,
		" min=-180 max=180 step=5 group=Trajectory");

	TwAddVarRW(bar, "Via Velocity:", TW_TYPE_FLOAT, &speed,
		" min=0 max=200 step=10 group=Trajectory");

	TwAddVarRW(bar, "Time:", TW_TYPE_FLOAT, &trajTime,
		" min=1 max=10 step=1 group=Trajectory");

	TwAddButton(bar, "Go", button_callback, NULL, "group=Trajectory");

	TwDefine(" Settings/Trajectory opened=false ");

	//===========Other==============
	TwAddSeparator(bar, "separator2", NULL);

	TwAddVarRW(bar, "Zoom:", TW_TYPE_FLOAT, &zoom,
		" min=0.1 max=10.0 step=0.1");


	// Send the new window size to AntTweakBar
	TwWindowSize(WINDOW_HEIGHT, WINDOW_WIDTH);
}
#endif

void Terminate()
{
#ifdef TW_INCLUDED
	TwTerminate();
#endif
}

int main(int argc, char **argv)
{
	// Initialize GLUT and process user parameters
	glutInit(&argc, argv);

	// Request double buffered true color window with Z-buffer
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	glutInitWindowSize(WINDOW_HEIGHT, WINDOW_WIDTH);
	glutInitWindowPosition(100, 0);

	// Create window
	glutCreateWindow("Project");

#ifdef TW_INCLUDED
	initTweak();
#endif

	atexit(Terminate);  // Called after glutMainLoop ends

	// Enable Z-buffer depth test
	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);

	glLoadIdentity();

	glOrtho(-WINDOW_WIDTH, WINDOW_WIDTH, -WINDOW_WIDTH, WINDOW_WIDTH, -WINDOW_WIDTH, WINDOW_WIDTH);

	// Callback functions
	glutDisplayFunc(Draw);
	glutSpecialFunc(directionHandler);
	glutKeyboardFunc(keyDownHandler);
	glutKeyboardUpFunc(keyUpHandler);

	// Pass control to GLUT for events
	glutMainLoop();

	Terminate();

	return 0;
}
