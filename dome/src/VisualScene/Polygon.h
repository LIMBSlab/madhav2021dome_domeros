#ifndef POLYGON_H
#define POLYGON_H

#include "DomeShape.h"

class Polygon : public DomeShape {
	public:
        // Constructors
		Polygon(time_t);
		Polygon(time_t,int,double);
		Polygon(time_t,double,double,int,double);

        // Destructor
 //               ~Polygon(void);

        // Accessors
		int getN(void);
		double getA(void);

        // Member functions
                void genDisplayList(void);

	private:
		int n; 			//< Number of sides
		double a; 		//< Side length
};
#endif
