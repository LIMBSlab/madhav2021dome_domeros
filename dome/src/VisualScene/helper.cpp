#include "helper.h"

// Random int between a and b
int randi (int a, int b) {
	int r = a + rand()%(b-a);
	return r;
}

// Random double between a and b
double randd (double a, double b) {
	double r = ((double) rand()) / ((double) RAND_MAX);
	return a + r*(b-a);
}

// OpenGL text output
void textOutput(int x, int y, float r, float g, float b, string str) {

	glColor3f(r,g,b);
	glRasterPos2i(x,y);
	for (int i = 0; i < str.length(); i++) {
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, str[i]);
	}

}
