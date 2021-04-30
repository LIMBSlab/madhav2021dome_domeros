/// General shape class


#ifndef SHAPE_H
#define SHAPE_H

#include "DomeShape.h"

class Shape : public DomeShape {
	public:
        // Constructors
		Shape(void);
		Shape(double,double,double);

        // Destructor
 //               ~Shape(void);

        // Accessors
                int setParams(vector<double>);

        // Member functions
		void render(time_t);

	private:
		time_t birthTime;
		time_t doomTime;
		static const int fadeInTime = 5000;
		static const int fadeOutTime = 5000;
                GLuint dlIdx;
};
#endif
