#pragma once
#include <GL/gl.h>
#include <GL/glut.h>
#include <math.h>
#include <time.h>
#include "DomeShape.h"
#include <vector>
#include <string>
#include <map>

#define PI 3.1415926535897932384626433832795

using namespace std;

class DomeShapeLibrary{
	public:
		static DomeShape* Shape(string, time_t);
};

class VerticalBarShape : public DomeShape {
	public:
		VerticalBarShape(time_t);
		void makeList();
};

class XShape : public DomeShape {
	public:
		XShape(time_t);
		void makeList();
};

class PlusShape : public DomeShape {
	public:
		PlusShape(time_t);
		void makeList();
};

class SquareDonutShape : public DomeShape {
	public:
		SquareDonutShape(time_t);
		void makeList();
};

class DiscShape : public DomeShape {
	public:
		DiscShape(time_t);
		void makeList();
};

class HorizontalEllipseShape: public DomeShape {
	public:
		HorizontalEllipseShape(time_t);
		void makeList();
};

class ThinStripeShape: public DomeShape {
	public:
		ThinStripeShape(time_t);
		void makeList();
};

class StripesShape: public DomeShape {
	public:
		StripesShape(time_t);
		void makeList();
};

class PizzaShape: public DomeShape {
	public:
		PizzaShape(time_t);
		void makeList();
};

class BandShape: public DomeShape {
	public:
		BandShape(time_t);
		void makeList();
};

class DotShape: public DomeShape {
	public:
		DotShape(time_t);
		void makeList();
};

class GridShape: public DomeShape {
	public:
		GridShape(time_t);
		void makeList();
};

class TeapotShape: public DomeShape {
	public:
		TeapotShape(time_t);
		void makeList();
};
