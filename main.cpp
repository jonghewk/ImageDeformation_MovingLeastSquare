/*
 *	Image Deformation Using Moving Least Square 
 *
 *  main.cpp
 *  Based on "Drawing plain and deformed grids in OpenGL"
 */

#define GLEW_STATIC

#define VIDEO 1
#define IMAGE 0 

#define PI 3.14159265
#define GRID_SCALE 6.0
#define X_POINT_COUNT 40
#define Y_POINT_COUNT 40
#define X_SPACING 0.2
#define Y_SPACING 0.2



#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <GL/glut.h>
#include <iostream>
#include <ctype.h>
#include <cmath>
#include <fstream>
#include "MovingLeastSquare.h"
#include <GL/glui.h>

using namespace cv;
using namespace std;

Point2f select;
vector<Point2f> selectPointsP;
vector<Point2f> selectPointsQ;
int click = 0;
float width;
float height;
int obj;
int nDeform = 0;

IplImage* image=0; 
int showpoints = 1;
int mainWindow;
int outputWindow;
/// On mouse event 
static void onMouse(int button, int event, int x, int y )
{
   	if(button == GLUT_LEFT_BUTTON && event == GLUT_DOWN)
	{
		if(nDeform==0)
		{
			width  = glutGet(GLUT_WINDOW_WIDTH);
			height  = glutGet(GLUT_WINDOW_HEIGHT);
			select = Point2f(GRID_SCALE*(x/(width/2.0)-1),-GRID_SCALE*(y/(height/2.0)-1));
			selectPointsP.push_back(select);
			click = 1;
		}
	}

	if(button == GLUT_RIGHT_BUTTON && event == GLUT_DOWN)
	{
		if(nDeform==0)
		{
			width  = glutGet(GLUT_WINDOW_WIDTH);
			height  = glutGet(GLUT_WINDOW_HEIGHT);
			select = Point2f(6.0*(x/(width/2.0)-1),-6.0*(y/(height/2.0)-1));
			selectPointsQ.push_back(select);
			click = 1;
		}
	}

	glutPostRedisplay();
}

GLdouble * computePointCoordinates(GLuint pointIndex,
								   GLuint xPointCount, GLuint yPointCount,
								   GLdouble xSpacing, GLdouble ySpacing)
{
	static GLdouble pt[3];

	GLdouble width = xSpacing * (xPointCount-1); 
	GLdouble height = ySpacing * (yPointCount-1); 
	GLdouble minX = -width/2;
	GLdouble minY = -height/2;
	pt[0] = minX + xSpacing * (pointIndex % xPointCount); 
	pt[1] = minY + ySpacing * (pointIndex / xPointCount); 
	pt[2] = 0;

	return pt;
}


void computePointCoordinate(int pointIndex,
							int xPointCount, int yPointCount,
							double xSpacing, double ySpacing, double pt[3])
{
	double * tmp = computePointCoordinates(pointIndex,
		xPointCount, yPointCount,
		xSpacing, ySpacing);

	pt[0] = tmp[0];
	pt[1] = tmp[1];
	pt[2] = tmp[2];
}

// renderGrid points (normal grid)
void renderGrid(GLuint xPointCount,GLuint yPointCount, GLdouble xSpacing, GLdouble ySpacing)
{
	double width = (xPointCount - 1) * xSpacing;
	double height = (yPointCount - 1) * ySpacing;
	double minX = -width/2;
	double minY = -height/2;

	GLuint nrQuads = (xPointCount-1)*(yPointCount-1);
	glBegin(GL_TRIANGLES); 
	for(GLuint i=0; i<nrQuads; i++) 
	{
		GLuint k = i + i/(xPointCount-1);
		GLuint a = k;
		GLuint b = k+1;
		GLuint c = k+1+xPointCount;
		GLuint d = k+xPointCount;
		GLdouble aPt[3], bPt[3], cPt[3], dPt[3]; 
		computePointCoordinate(a, xPointCount, yPointCount,xSpacing, ySpacing, aPt);
		computePointCoordinate(b, xPointCount, yPointCount,xSpacing, ySpacing, bPt); 
		computePointCoordinate(c, xPointCount, yPointCount,xSpacing, ySpacing, cPt); 
		computePointCoordinate(d, xPointCount, yPointCount,xSpacing, ySpacing, dPt);
		// Triangle 1
		glTexCoord2f((aPt[0] - minX) / width, -(aPt[1] - minY) / height);
		glVertex3dv(aPt);
		glTexCoord2f((cPt[0] - minX) / width, -(cPt[1] - minY) / height);
		glVertex3dv(cPt);
		glTexCoord2f((dPt[0] - minX) / width, -(dPt[1] - minY) / height);
		glVertex3dv(dPt);
		// Triangle 2
		glTexCoord2f((aPt[0] - minX) / width, -(aPt[1] - minY) / height);
		glVertex3dv(aPt);
		glTexCoord2f((bPt[0] - minX) / width, -(bPt[1] - minY) / height);
		glVertex3dv(bPt);
		glTexCoord2f((cPt[0] - minX) / width, -(cPt[1] - minY) / height);
		glVertex3dv(cPt);
	}
	glEnd();
}

//rendering & displaying the deformed grid
void renderDeformedGrid(GLuint xPointCount,GLuint yPointCount, GLdouble xSpacing, GLdouble ySpacing, Mat fv)
{
	
	double width = (xPointCount - 1) * xSpacing;
	double height = (yPointCount - 1) * ySpacing;
	double minX = -width/2;
	double minY = -height/2;
	glBegin(GL_TRIANGLES); 
	glColor3f(0.0,0.0,0.0);
	int nrQuads = (xPointCount-1)*(yPointCount-1);
	for(int i=0; i<nrQuads; i++) 
	{
		int k = i + i/(xPointCount-1);
		int a = k;
		int b = k+1;
		int c = k+1+xPointCount;
		int d = k+xPointCount;
		double aPt[3], bPt[3], cPt[3], dPt[3]; 
		double aPtIm[3], bPtIm[3], cPtIm[3], dPtIm[3];
		computePointCoordinate(a, xPointCount, yPointCount, xSpacing, ySpacing, aPtIm);
		computePointCoordinate(b, xPointCount, yPointCount, xSpacing, ySpacing, bPtIm);
		computePointCoordinate(c, xPointCount, yPointCount, xSpacing, ySpacing, cPtIm);
		computePointCoordinate(d, xPointCount, yPointCount, xSpacing, ySpacing, dPtIm);
		
		//get the deformed points 
		aPt[0] = fv.at<float>(0, a);
		aPt[1] = fv.at<float>(1, a);
		aPt[2] = 0;

		bPt[0] = fv.at<float>(0, b);
		bPt[1] = fv.at<float>(1, b);
		bPt[2] = 0;

		cPt[0] = fv.at<float>(0, c);
		cPt[1] = fv.at<float>(1, c);
		cPt[2] = 0;

		dPt[0] = fv.at<float>(0, d);
		dPt[1] = fv.at<float>(1, d);
		dPt[2] = 0;
		//mapping the actual points
		glTexCoord2f((aPtIm[1] - minX) / width, -(aPtIm[0] - minY) / height);
		//draw the deformed points
		glVertex3dv(aPt);
		glTexCoord2f((cPtIm[1] - minX) / width, -(cPtIm[0] - minY) / height);
		glVertex3dv(cPt);
		glTexCoord2f((dPtIm[1] - minX) / width, -(dPtIm[0] - minY) / height);
		glVertex3dv(dPt);

		glTexCoord2f((aPtIm[1] - minX) / width, -(aPtIm[0] - minY) / height);
		glVertex3dv(aPt);
		glTexCoord2f((bPtIm[1] - minX) / width, -(bPtIm[0] - minY) / height);
		glVertex3dv(bPt);
		glTexCoord2f((cPtIm[1] - minX) / width, -(cPtIm[0] - minY) / height);
		glVertex3dv(cPt);
	}
	glEnd();
}

//render all grid points for calculation
Mat render(int xPointCount, int yPointCount, double xSpacing, double ySpacing)
{
	//change the size of the grid
	Mat a = Mat::zeros(2,X_POINT_COUNT*Y_POINT_COUNT,CV_32F);
	double width = xSpacing * (xPointCount-1); 
	double height = ySpacing * (yPointCount-1); 
	double minX = -width/2;
	double minY = -height/2;
	glBegin(GL_POINTS);
	for(int i=0; i<xPointCount; i++) {
		for(int j=0; j<yPointCount; j++) {
			double x = minX + i*xSpacing;
			double y = minY + j*ySpacing;
			double z = 0;
			//if draw vertex of grid, uncomment below
			//glVertex3f(x, y, z);
			
			//save the mat points to get v
			a.at<float>(0, i*yPointCount + j) = (float)x;
			a.at<float>(1, i*yPointCount + j) = (float)y;
		}
	}
	glEnd();

	return a;
}
//texture map a image
void texturemapping(void)
{
	//glClearColor(1.0, 1.0, 1.0, 0.0);
	glShadeModel(GL_FLAT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_TEXTURE_2D);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glEnable(GL_DEPTH_TEST);
	
	GLuint texture;
	// Create Texture
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); 
	glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); 

	if(!image)
		return;

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image->width, image->height,0, GL_BGR_EXT, GL_UNSIGNED_BYTE, image->imageData);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

//draw the points
void drawPoints(Mat p, int color)
{
	glDisable(GL_TEXTURE_2D);
	glBegin(GL_POINTS);
	if(color ==0)
		glColor3f(0.5, 0.0, 1.0);
	else
		glColor3f(1.0, 0.0, 0.0);

	for (int i = 0; i < p.cols; i++)
		glVertex3f(p.at<float>(0, i) ,p.at<float>(1, i),2 );
	
	glEnd();
}

void display(void)
{
	/* clear all pixels  */
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//glTranslatef(0,0,-6);
	//glRotatef(-60,0,0,0);
	glColor3f(0.0, 0.0, 0.0);

	Mat v = render(X_POINT_COUNT,Y_POINT_COUNT,X_SPACING,Y_SPACING);   
	
	texturemapping();
	renderGrid(X_POINT_COUNT,Y_POINT_COUNT,X_SPACING,Y_SPACING);  
	
	Mat p; 
	Mat q; 
	
	if( click == 1)
	{
		if(showpoints==1)
		{
			glPointSize(6);

			for(int i =0 ; i<selectPointsP.size(); i++)
				drawPoints(Mat(selectPointsP.at(i)),0);
		
			for(int i =0 ; i<selectPointsQ.size(); i++)
				drawPoints(Mat(selectPointsQ.at(i)),1);
		}
	}

	glFinish();
	glutSwapBuffers();
}


void display2(void)
{
	/* clear all pixels  */
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//glTranslatef(0,0,-6);
	//glRotatef(-60,0,0,0);
	glColor3f(0.0, 0.0, 0.0);

	Mat v = render(X_POINT_COUNT,Y_POINT_COUNT,X_SPACING,Y_SPACING);   
	
	Mat p; 
	Mat q; 

	
	if(nDeform==1)
	{
		p = Mat::zeros(2,selectPointsP.size(),CV_32F);
		q = Mat::zeros(2,selectPointsQ.size(),CV_32F);
		//initializing p points for fish eye image
		for(int i =0 ; i<selectPointsP.size(); i++)
		{
			p.at<float>(0,i) =  (selectPointsP.at(i)).x;
			p.at<float>(1,i) =  (selectPointsP.at(i)).y;
		}
		//initializing q points for fish eye image
		for(int i =0 ; i<selectPointsQ.size(); i++)
		{
			q.at<float>(0,i) =  (selectPointsQ.at(i)).x;
			q.at<float>(1,i) =  (selectPointsQ.at(i)).y;
		}

		double a  = 2.0 ;
		double minX = 0; double minY = 0;

		//Precompute
		Mat w = precomputeWeights(p, v, a);
		Mat A;Mat fv;
		vector <_typeA> tA;
		typeRigid mlsd;

		switch(obj)
		{
		case 0:
			A = precomputeAffine(p, v, w);
			fv = PointsTransformAffine(w, A, q);
			break;

		case 1:
			tA = precomputeSimilar(p,v, w);
			fv = PointsTransformSimilar(w, tA, q);
			break;

		case 2:
			mlsd = precomputeRigid(p, v, w);
			fv = PointsTransformRigid(w,mlsd, q);
			break;

		default:
			break;
		}
		//texture mapping
		texturemapping();
		//render the deformed grid
		renderDeformedGrid(X_POINT_COUNT, Y_POINT_COUNT, X_SPACING, Y_SPACING, fv);
		//glutCreateWindow("Output");
	}

	if( click == 1)
	{
		if(showpoints==1)
		{
			glPointSize(6);

			for(int i =0 ; i<selectPointsP.size(); i++)
				drawPoints(Mat(selectPointsP.at(i)),0);
		
			for(int i =0 ; i<selectPointsQ.size(); i++)
				drawPoints(Mat(selectPointsQ.at(i)),1);
		}
	}

	glFinish();
	glutSwapBuffers();
}


void init (void) 
{
	/* select clearing color 	*/
	glClearColor (1.0, 1.0, 1.0,1.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glEnable( GL_POINT_SMOOTH );
	glPointSize( 3.0f );
}

void resize(int w, int h)
{
	glViewport(0,0, (GLsizei) w, (GLsizei) h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-GRID_SCALE, GRID_SCALE, -GRID_SCALE, GRID_SCALE, -2.0, 2.0);
}
//callbacks on button
void deform()
{
	nDeform = 1;
	glutSetWindow(outputWindow);
}
//callbacks for radio
void control_cb(int ID)
{
	
}

void control_cb_check(int ID)
{
	
}
#if VIDEO
VideoCapture cap; 
#endif

Mat frame;

void idle()
{
#if VIDEO
	cap>>frame;
	image=cvCloneImage(&(IplImage)frame);
	
	if(glutGetWindow() == mainWindow)
		glutSetWindow(outputWindow);
	else
		glutSetWindow(mainWindow);
#endif 
	glutPostRedisplay();
}
int main(int argc, char** argv)
{
#if VIDEO
	cap.open(0);
	if( !cap.isOpened() )
	{
		cout << "***Could not initialize capturing...***\n";
		cout << "Current parameter's value: \n";
		return -1;
	}
	cap>>frame;
#endif
	//load the image
#if IMAGE
	frame = imread("top_left.jpg");
#endif	
	image=cvCloneImage(&(IplImage)frame);
	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize (image->width, image->height); 
	glutInitWindowPosition (-1, -1);
	mainWindow = glutCreateWindow ("Input Window");
	init ();
	glutDisplayFunc(display); 
	glutMouseFunc(onMouse);
	glutReshapeFunc(resize);


	GLUI *glui = GLUI_Master.create_glui( "Image Deformation GUI" );
	GLUI_Master.set_glutIdleFunc(NULL);
	glui->add_statictext( "Left Click : P points" );
	glui->add_statictext( "Right Click : Q points" );
	glui->add_separator();
	glui->add_checkbox( "Show Points", &showpoints, 1, control_cb_check );
	glui->add_separator();
	GLUI_Panel *obj_panel = glui->add_panel( "Deform Type" );
	GLUI_RadioGroup *group1 =
	glui->add_radiogroup_to_panel(obj_panel,&obj,3,control_cb);
	glui->add_radiobutton_to_group( group1, "Affine" );
	glui->add_radiobutton_to_group( group1, "Similarity" );
	glui->add_radiobutton_to_group( group1, "Rigid" );
	glui->add_button("Deform",0,(GLUI_Update_CB)deform);
	glutAttachMenu(GLUT_RIGHT_BUTTON);   


	glutInitWindowSize (image->width, image->height); 
	glutInitWindowPosition (500, -1);
	outputWindow = glutCreateWindow ("Output Window");
	init ();
	glutDisplayFunc(display2); 
	glutMouseFunc(onMouse);
	glutReshapeFunc(resize);
	glutIdleFunc(idle);

	glutMainLoop();
	return 0; 
}








