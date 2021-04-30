#include "Polygon.h"
#include "GL/gl.h"
#include <GL/glu.h>
#include "helper.h"
#include <cmath>
#include <ctime>

#define PI 3.1415926535897932384626433832795

/******* CONSTRUCTORS ********/
Polygon::Polygon (time_t time1) : DomeShape(time1) {
	n = randi(3,10);
	a = randd(0.5,5);	
        
        genDisplayList();
}

Polygon::Polygon (time_t time1, int n_i, double a_i) : DomeShape(time1) {
	if (n_i<3) n=3;
	else n = n_i;
	a = a_i;	

        genDisplayList();
}

Polygon::Polygon (time_t time1, double th_i, double ph_i, int n_i, double a_i) : DomeShape(time1,th_i,ph_i) {
	if (n_i<3) n=3;
	else n = n_i;
	a = a_i;	

        genDisplayList();
}

/******* DESTRUCTOR ********/
/*
Polygon::~Polygon (void)
{
    glDeleteLists(dlIdx,1);
}
*/

/******* ACCESSORS ********/
int Polygon::getN(void) {
	return n;
	}

double Polygon::getA(void) {
	return a;
	}

/******* MEMBER FUNCTIONS ********/
void Polygon::genDisplayList(void) {
        dlIdx = glGenLists(1);
        glNewList(dlIdx, GL_COMPILE);
            glPushMatrix();
            
            moveToDome();
            glBegin(GL_POLYGON);
            double ang;
                    for (double i=0;i<n;i++) {
                            ang = PI/2 + PI/n + i*2*PI/n;
                            glVertex3f(a/sqrt(n)*cos(ang),a/sqrt(n)*sin(ang),0);
                    }
            glEnd();

            glPopMatrix();
        glEndList();
}
