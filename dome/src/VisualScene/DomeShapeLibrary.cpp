/** \file DomeShapeLibrary.cpp
 * 
 * Library containing functions for generating standard shapes for the dome.
 *
 * \author Manu S. Madhav
 * \date 08-Dec-2015
 * 
 * */

#include "DomeShapeLibrary.h"

DomeShape* DomeShapeLibrary::Shape(string shapeName, time_t time1) {
    if (!shapeName.compare("vertical_bar")) return (new VerticalBarShape(time1));
    if (!shapeName.compare("x")) return (new XShape(time1));
    if (!shapeName.compare("plus")) return (new PlusShape(time1));
    if (!shapeName.compare("square_donut")) return (new SquareDonutShape(time1));
    if (!shapeName.compare("disc")) return (new DiscShape(time1));
    if (!shapeName.compare("horizontal_ellipse")) return (new HorizontalEllipseShape(time1));
    if (!shapeName.compare("thin_stripe")) return (new ThinStripeShape(time1));
    if (!shapeName.compare("stripes")) return (new StripesShape(time1));
    if (!shapeName.compare("pizza")) return (new PizzaShape(time1));
    if (!shapeName.compare("band")) return (new BandShape(time1));
    if (!shapeName.compare("dot")) return (new DotShape(time1));
    if (!shapeName.compare("grid")) return (new GridShape(time1));
    if (!shapeName.compare("teapot")) return (new TeapotShape(time1));
    else return (new TeapotShape(time1)); // Make this a default shape
}

VerticalBarShape::VerticalBarShape(time_t time1) : DomeShape(time1) {}
void VerticalBarShape::makeList() {
    if (glIsList(dlIdxAlpha)) glDeleteLists(dlIdxAlpha,1);
    if (glIsList(dlIdxBlack)) glDeleteLists(dlIdxBlack,1);

    double w = 40, h = 120;
    setColor(0.25,0.25,0.25);
    
    // Make displayList
    GLuint dlIdx = glGenLists(1);
    glNewList(dlIdx, GL_COMPILE);
        glPushMatrix();
        moveToDome();

        glBegin(GL_POLYGON);
            glVertex3f(-w/2,-h/2,0);
            glVertex3f(w/2,-h/2,0);
            glVertex3f(w/2,h/2,0);
            glVertex3f(-w/2,h/2,0);
            glVertex3f(-w/2,-h/2,0);
        glEnd();

        glPopMatrix();
    glEndList();
    setAlphaDisplayList(dlIdx);
}

XShape::XShape(time_t time1) : DomeShape(time1) {}
void XShape::makeList() {
    if (glIsList(dlIdxAlpha)) glDeleteLists(dlIdxAlpha,1);
    if (glIsList(dlIdxBlack)) glDeleteLists(dlIdxBlack,1);

    double i;
    GLuint dlIdx;
    double l = 70, w = 20; // Length and width of X

    setColor(0.85,0.85,0.85);

    dlIdx = glGenLists(1);
    glNewList(dlIdx, GL_COMPILE);
        glPushMatrix();
        moveToDome();
        glRotatef(45,0.0,0.0,1.0);

        glBegin(GL_POLYGON);
            glVertex3f(-w/2,-l/2,0);
            glVertex3f(w/2,-l/2,0);
            glVertex3f(w/2,l/2,0);
            glVertex3f(-w/2,l/2,0);
            glVertex3f(-w/2,-l/2,0);
        glEnd();
        glBegin(GL_POLYGON);
            glVertex3f(-l/2,-w/2,0);
            glVertex3f(l/2,-w/2,0);
            glVertex3f(l/2,w/2,0);
            glVertex3f(-l/2,w/2,0);
            glVertex3f(-l/2,-w/2,0);
        glEnd();

        glPopMatrix();
    glEndList();
    setAlphaDisplayList(dlIdx);
}

PlusShape::PlusShape(time_t time1) : DomeShape(time1) {}
void PlusShape::makeList() {
    if (glIsList(dlIdxAlpha)) glDeleteLists(dlIdxAlpha,1);
    if (glIsList(dlIdxBlack)) glDeleteLists(dlIdxBlack,1);

    double i;
    GLuint dlIdx;
    double l = 60, w = 30; // Length and width of plus arm

    setColor(0.6,0.6,0.6);

    dlIdx = glGenLists(1);
    glNewList(dlIdx, GL_COMPILE);
        glPushMatrix();
        moveToDome();

        glBegin(GL_POLYGON);
            glVertex3f(-w/2,-l/2,0);
            glVertex3f(w/2,-l/2,0);
            glVertex3f(w/2,l/2,0);
            glVertex3f(-w/2,l/2,0);
            glVertex3f(-w/2,-l/2,0);
        glEnd();
        glBegin(GL_POLYGON);
            glVertex3f(-l/2,-w/2,0);
            glVertex3f(l/2,-w/2,0);
            glVertex3f(l/2,w/2,0);
            glVertex3f(-l/2,w/2,0);
            glVertex3f(-l/2,-w/2,0);
        glEnd();

        glPopMatrix();
    glEndList();
    setAlphaDisplayList(dlIdx);
}


SquareDonutShape::SquareDonutShape(time_t time1) : DomeShape(time1) {}
void SquareDonutShape::makeList() {
    if (glIsList(dlIdxAlpha)) glDeleteLists(dlIdxAlpha,1);
    if (glIsList(dlIdxBlack)) glDeleteLists(dlIdxBlack,1);

    GLuint dlIdxAlpha,dlIdxBlack;
    double w_out = 80, h_out = 80;
    double w_in = 40, h_in = 40;

    setColor(0.4,0.4,0.4);
    
    // Make displayList
    dlIdxAlpha = glGenLists(1);
    glNewList(dlIdxAlpha, GL_COMPILE);
        glPushMatrix();
        moveToDome();

        glBegin(GL_POLYGON);
            glVertex3f(-w_out/2,-h_out/2,0);
            glVertex3f(w_out/2,-h_out/2,0);
            glVertex3f(w_out/2,h_out/2,0);
            glVertex3f(-w_out/2,h_out/2,0);
            glVertex3f(-w_out/2,-h_out/2,0);
        glEnd();

        glPopMatrix();
    glEndList();
    setAlphaDisplayList(dlIdxAlpha);

    dlIdxBlack = glGenLists(1);
    glNewList(dlIdxBlack, GL_COMPILE);
        glPushMatrix();
        moveToDome();

        glBegin(GL_POLYGON);
            glVertex3f(-w_in/2,-h_in/2,0);
            glVertex3f(w_in/2,-h_in/2,0);
            glVertex3f(w_in/2,h_in/2,0);
            glVertex3f(-w_in/2,h_in/2,0);
            glVertex3f(-w_in/2,-h_in/2,0);
        glEnd();

        glPopMatrix();
    glEndList();
    setBlackDisplayList(dlIdxBlack);
}


DiscShape::DiscShape(time_t time1) : DomeShape(time1) {}
void DiscShape::makeList() {
    if (glIsList(dlIdxAlpha)) glDeleteLists(dlIdxAlpha,1);
    if (glIsList(dlIdxBlack)) glDeleteLists(dlIdxBlack,1);

    double i;
    GLuint dlIdx;
    double r = 50; // Circle radius

    setColor(0.7,0.7,0.7);

    dlIdx = glGenLists(1);
    glNewList(dlIdx, GL_COMPILE);
        glPushMatrix();
        moveToDome();

        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0,0,0);
        for (i=0;i<=2*PI;i+=PI/20) {
            glVertex3f(r*cos(i),r*sin(i),0);
        }
        glEnd();

        glPopMatrix();
    glEndList();
    setAlphaDisplayList(dlIdx);

}

HorizontalEllipseShape::HorizontalEllipseShape(time_t time1) : DomeShape(time1) {}
void HorizontalEllipseShape::makeList() {
    if (glIsList(dlIdxAlpha)) glDeleteLists(dlIdxAlpha,1);
    if (glIsList(dlIdxBlack)) glDeleteLists(dlIdxBlack,1);

    double i;
    GLuint dlIdxAlpha,dlIdxBlack;
    double c = 30, d = 15; //*< Inner ellipse params
    double a = 60, b = 30; // Outer ellipse params

    setColor(0.4,0.4,0.4);

    dlIdxAlpha = glGenLists(1);
    glNewList(dlIdxAlpha, GL_COMPILE);
        glPushMatrix();
        moveToDome();

        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0,0,0);
        for (i=0;i<=2*PI;i+=PI/20) {
            glVertex3f(a*cos(i),b*sin(i),0);
        }
        glEnd();

        glPopMatrix();
    glEndList();
    setAlphaDisplayList(dlIdxAlpha);

    dlIdxBlack = glGenLists(1);
    glNewList(dlIdxBlack, GL_COMPILE);
        glPushMatrix();
        moveToDome();

        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0,0,0);
        for (i=0;i<=2*PI;i+=PI/20) {
            glVertex3f(c*cos(i),d*sin(i),0);
        }
        glEnd();
        
        glPopMatrix();
    glEndList();
    setBlackDisplayList(dlIdxBlack);
}


ThinStripeShape::ThinStripeShape(time_t time1) : DomeShape(time1) {}
void ThinStripeShape::makeList() {
    if (glIsList(dlIdxAlpha)) glDeleteLists(dlIdxAlpha,1);
    if (glIsList(dlIdxBlack)) glDeleteLists(dlIdxBlack,1);

    GLuint dlIdx;
    double w = 1;
    double angH = 60*PI/180;
    double h = DomeShape::R*angH;

    setColor(1.0,1.0,1.0);
    
    // Make displayList
    dlIdx = glGenLists(1);
    glNewList(dlIdx, GL_COMPILE);
        glPushMatrix();
        moveToDome();

        glBegin(GL_POLYGON);
            glVertex3f(-w/2,-h/2,0);
            glVertex3f(w/2,-h/2,0);
            glVertex3f(w/2,h/2,0);
            glVertex3f(-w/2,h/2,0);
            glVertex3f(-w/2,-h/2,0);
        glEnd();

        glPopMatrix();
    glEndList();
    setAlphaDisplayList(dlIdx);
}


/** Returns a equally spaced set of radial stripes around the dome.
 *
 *  \param birthTime Time of instantiation.
 *  \return vector, each stripe is a DomeShape object.
 *  **/
StripesShape::StripesShape(time_t time1) : DomeShape(time1) {}
void StripesShape::makeList() {
    if (glIsList(dlIdxAlpha)) glDeleteLists(dlIdxAlpha,1);
    if (glIsList(dlIdxBlack)) glDeleteLists(dlIdxBlack,1);

    GLuint dlIdx;
    int nStripes = 80;
    double angH = 40*PI/180;
    double w = 2*PI*DomeShape::R/(nStripes*3);
    double h = DomeShape::R*angH;

    setColor(0.5,0.5,0.5);
    
    // Make displayList
    dlIdx = glGenLists(1);
    glNewList(dlIdx, GL_COMPILE);
        for(int i=0; i<nStripes; i++) {
            setTheta(i*2*PI/nStripes);
            glPushMatrix();
            moveToDome();

            glBegin(GL_POLYGON);
                glVertex3f(-w/2,-h/2,0);
                glVertex3f(w/2,-h/2,0);
                glVertex3f(w/2,h/2,0);
                glVertex3f(-w/2,h/2,0);
                glVertex3f(-w/2,-h/2,0);
            glEnd();

            glPopMatrix();
        }
    glEndList();

    setAlphaDisplayList(dlIdx);
}

PizzaShape::PizzaShape(time_t time1) : DomeShape(time1) {}
void PizzaShape::makeList() {
    if (glIsList(dlIdxAlpha)) glDeleteLists(dlIdxAlpha,1);
    if (glIsList(dlIdxBlack)) glDeleteLists(dlIdxBlack,1);

    double i;
    GLuint dlIdxAlpha,dlIdxBlack;
    double p = 50, q = 100, r = 7;

    setColor(0.35,0.35,0.35);

    dlIdxAlpha = glGenLists(1);
    glNewList(dlIdxAlpha, GL_COMPILE);
        glPushMatrix();
        moveToDome();

        glBegin(GL_POLYGON);
            glVertex3f(0,-q/2,0);
            glVertex3f(-p/2,q/2,0);
            glVertex3f(p/2,q/2,0);
        glEnd();
        
        glPopMatrix();
    glEndList();
    setAlphaDisplayList(dlIdxAlpha);

    dlIdxBlack = glGenLists(1);
    glNewList(dlIdxBlack, GL_COMPILE);
        glPushMatrix();
        moveToDome();

        glBegin(GL_POLYGON);
            for (i=0;i<2*PI;i+=PI/5) {
                glVertex3f(0.25*p + r*cos(i),0.35*q + r*sin(i),0);
            }
        glEnd();

        glBegin(GL_POLYGON);
            for (i=0;i<2*PI;i+=PI/5) {
                glVertex3f(-0.10*p + r*cos(i),0.15*q + r*sin(i),0);
            }
        glEnd();

        glBegin(GL_POLYGON);
            for (i=0;i<2*PI;i+=PI/5) {
                glVertex3f(0 + r*cos(i),-0.15*q + r*sin(i),0);
            }
        glEnd();

        glPopMatrix();
    glEndList();
    setBlackDisplayList(dlIdxBlack);
}

BandShape::BandShape(time_t time1) : DomeShape(time1) {}
void BandShape::makeList() {
    if (glIsList(dlIdxAlpha)) glDeleteLists(dlIdxAlpha,1);
    if (glIsList(dlIdxBlack)) glDeleteLists(dlIdxBlack,1);

    double i;
    GLuint dlIdxAlpha,dlIdxBlack;
    double rBandOuter=110,rBandInner=100;

    setColor(0.4,0.4,0.4);

    dlIdxAlpha = glGenLists(1);
    glNewList(dlIdxAlpha, GL_COMPILE);
        glPushMatrix();
        moveToDome();

        glBegin(GL_POLYGON);
            for (i=0;i<2*PI;i+=PI/100) {
                glVertex3f(rBandOuter*cos(i),rBandOuter*sin(i),0);
            }
        glEnd();
        
        glPopMatrix();
    glEndList();
    setAlphaDisplayList(dlIdxAlpha);

    dlIdxBlack = glGenLists(1);
    glNewList(dlIdxBlack, GL_COMPILE);
        glPushMatrix();
        moveToDome();

        glBegin(GL_POLYGON);
            for (i=0;i<2*PI;i+=PI/100) {
                glVertex3f(rBandInner*cos(i),rBandInner*sin(i),0);
            }
        glEnd();
        
        glPopMatrix();
    glEndList();
    setBlackDisplayList(dlIdxBlack);
}

DotShape::DotShape(time_t time1) : DomeShape(time1) {}
void DotShape::makeList() {
    if (glIsList(dlIdxAlpha)) glDeleteLists(dlIdxAlpha,1);
    if (glIsList(dlIdxBlack)) glDeleteLists(dlIdxBlack,1);

    double r = 3.0;
    setColor(0.4,0.4,0.4);

    GLuint dlIdxAlpha = glGenLists(1);
        glNewList(dlIdxAlpha, GL_COMPILE);
        glPushMatrix();

        moveToDome();
        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0,0,0);
        for (double i=0;i<=2*PI;i+=PI/5) {
            glVertex3f(r*cos(i),r*sin(i),0);
        }
        glEnd();

        glPopMatrix();
    glEndList();
    setAlphaDisplayList(dlIdxAlpha);
}

GridShape::GridShape(time_t time1) : DomeShape(time1) {}
void GridShape::makeList() {
    if (glIsList(dlIdxAlpha)) glDeleteLists(dlIdxAlpha,1);
    if (glIsList(dlIdxBlack)) glDeleteLists(dlIdxBlack,1);

    setColor(0.6,0.6,0.6);

    GLuint dlIdxAlpha = glGenLists(1);
        glNewList(dlIdxAlpha, GL_COMPILE);
        glPushMatrix();

        moveToDome();
        glutWireSphere(220,100,100); 

        glPopMatrix();
    glEndList();
    setAlphaDisplayList(dlIdxAlpha);
}

TeapotShape::TeapotShape(time_t time1) : DomeShape(time1) {}
void TeapotShape::makeList() {
    if (glIsList(dlIdxAlpha)) glDeleteLists(dlIdxAlpha,1);
    if (glIsList(dlIdxBlack)) glDeleteLists(dlIdxBlack,1);

    setColor(1.0,1.0,1.0);

    GLuint dlIdxAlpha = glGenLists(1);
        glNewList(dlIdxAlpha, GL_COMPILE);
        glPushMatrix();

        moveToDome();
        glutWireTeapot(30); 

        glPopMatrix();
    glEndList();
    setAlphaDisplayList(dlIdxAlpha);
}
