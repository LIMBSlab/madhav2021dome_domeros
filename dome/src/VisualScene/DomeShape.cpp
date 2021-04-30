#include "DomeShape.h"
#include "helper.h"
#include <math.h>

#define PI 3.1415926535897932384626433832795
using namespace std;

/******* CONSTRUCTORS ********/
DomeShape::DomeShape (time_t time1) {
    birthTime = time1;
    mode = DomeShapeMode::OFF;
    type = "generic";
    name = "unnamed";

    fadeInTime = 0;
    fadeOutTime = 0;
    
    hasAlphaList = 0;
    hasBlackList = 0;

    cycleProbability = 0.0;
    
    randomizePosition();
}

DomeShape::DomeShape (time_t time1, double th_i, double ph_i) : DomeShape(time1) {
    if (th_i < 0) th = 0;
    else if (th_i > 2 * PI) th = 2*PI;
    else th = th_i;

    if (ph_i < 0) ph = 0;
    else if (ph_i > PI) ph = PI;
    else ph = ph_i;
}

/******* ACCESSORS ********/
void DomeShape::randomizePosition() {
    th = randd(0,2*PI);
    ph = asin(randd(0,sin(60*PI/180)));
}

int DomeShape::setColor(double r, double g, double b) {
    if (r >= 0.0 && r <= 1.0 && g >= 0.0 && g <= 1.0 && b >= 0.0 && b <= 1.0) {
        color[0] = r;
        color[1] = g;
        color[2] = b;
        return 0;
    }
    else return 1;
}

int DomeShape::setColor(double w) {
    // Grayscale color
    if (w >= 0.0 && w <= 1.0) {
        color[0] = w;
        color[1] = w;
        color[2] = w;
        return 0;
    }
    else return 1;
}

int DomeShape::setFadeIn(double fade_in_time) {
    if (fade_in_time >= 0.0) {
        fadeInTime = fade_in_time;
        return 0;
    }
    else return 1;
}

int DomeShape::setFadeOut(double fade_out_time) {
    if (fade_out_time >= 0.0) {
        fadeOutTime = fade_out_time;
        return 0;
    }
    else return 1;
}

int DomeShape::setAlphaDisplayList(GLuint dlIdx_i) {
    if (glIsList(dlIdx_i)) {
        dlIdxAlpha = dlIdx_i;
        hasAlphaList = 1;
        return 0;
    }
    else return 1;
}

int DomeShape::setBlackDisplayList(GLuint dlIdx_i) {
    if (glIsList(dlIdx_i)) {
        dlIdxBlack = dlIdx_i;
        hasBlackList = 1;
        return 0;
    }
    else return 1;
}

int DomeShape::setCycleProbability(double prob) {
    if (prob >= 0.0 && prob <= 1.0) {
        cycleProbability = prob;
        return 0;
    }
    else return 1;
}

/******* MEMBER FUNCTIONS ********/
// Move model matrix to a point in the dome
void DomeShape::moveToDome(void) {
    double M[16] = {-sin(th),cos(th),0,0,
        cos(th)*sin(ph),sin(th)*sin(ph),-cos(ph),0,
        cos(th)*cos(ph),sin(th)*cos(ph),sin(ph),0,
        R*cos(ph)*cos(th),R*cos(ph)*sin(th),-R*sin(ph),1};
    glMultMatrixd(M);
}

void DomeShape::setMode(DomeShapeMode mode1) {
    mode = mode1;
}


bool DomeShape::render(time_t time1) {
	double alpha;
    bool modeChangedFlag = 0;

    switch (mode) {
        case DomeShapeMode::DISAPPEAR:
            alpha = 0.0;
            mode = DomeShapeMode::OFF; modeChangedFlag = 1;
        case DomeShapeMode::OFF: ///< Invisible
            alpha = 0.0;
            break;
		case DomeShapeMode::START_VISIBLE: ///< Appear
            birthTime = time1;
            if (fadeInTime) {
                alpha = 0.0;
                mode = DomeShapeMode::FADE_IN; modeChangedFlag = 1;
            }
            else {
                alpha = 1.0;
                mode = DomeShapeMode::ON; modeChangedFlag = 1;
            }
            break;
        case DomeShapeMode::FADE_IN: ///< Fade in
            alpha = (double)(time1-birthTime)/fadeInTime;
            if (alpha >= 1.0) {
                alpha = 1.0;
                mode = DomeShapeMode::ON; modeChangedFlag = 1;
            }
            break;
        case DomeShapeMode::APPEAR:
            alpha = 1.0;
            mode = DomeShapeMode::ON; modeChangedFlag = 1;
		case DomeShapeMode::ON: ///< Full brightness
           alpha = 1.0;
           if (cycleProbability > 0.0 && randd(0.0, 1.0)<cycleProbability) mode = DomeShapeMode::CYCLE_START;
           break;
		case DomeShapeMode::STOP_VISIBLE: ///< Disappear 
            doomTime = time1;
            if (fadeOutTime) {
                alpha = 1.0;
                mode = DomeShapeMode::FADE_OUT; modeChangedFlag = 1;
            }
            else {
                alpha = 0.0;
                mode = DomeShapeMode::OFF; modeChangedFlag = 1;
            }
            break;
        case DomeShapeMode::FADE_OUT: ///< Fade out
            alpha = 1.0 - (double)(time1-doomTime)/fadeOutTime;
            if (alpha <= 0.0) {
                alpha = 0.0;
                mode = DomeShapeMode::OFF; modeChangedFlag = 1;
            }
            break;
        case DomeShapeMode::CYCLE_START:
            alpha = 1.0;
            doomTime = time1;
            mode = DomeShapeMode::CYCLE_OUT; modeChangedFlag = 1;
        case DomeShapeMode::CYCLE_OUT:
            alpha = 1.0 - (double)(time1-doomTime)/fadeOutTime;
            if (alpha <= 0.0) {
                alpha = 0.0;
                birthTime = time1;
                randomizePosition();
                makeList();
                mode = DomeShapeMode::CYCLE_IN; modeChangedFlag = 1;
            }
            break;
        case DomeShapeMode::CYCLE_IN:
            alpha = (double)(time1-birthTime)/fadeInTime;
            if (alpha >= 1.0) {
                alpha = 1.0;
                mode = DomeShapeMode::ON; modeChangedFlag = 1;
            }
            break;
        default:
            alpha = 0.0;
            mode = DomeShapeMode::OFF; modeChangedFlag = 1;
    }

    if (alpha>0.0) {
        if (hasAlphaList) {
            glColor4f(color[0],color[1],color[2],alpha);
            glCallList(dlIdxAlpha);
        }
        if (hasBlackList) {
            glColor4f(0.0,0.0,0.0,alpha);
            glCallList(dlIdxBlack);
        }
        glColor4f(1.0,1.0,1.0,1.0);
    }

    return modeChangedFlag;
}

void DomeShape::fadeOnOff(bool on) {
    (on)? fadeOn() : fadeOff();
}

void DomeShape::turnOnOff(bool on) {
    (on)? turnOn() : turnOff();
}

void DomeShape::fadeOn(void) {
    mode = DomeShapeMode::START_VISIBLE;
}

void DomeShape::turnOn(void) {
    mode = DomeShapeMode::APPEAR;
}

void DomeShape::fadeOff(void) {
    mode = DomeShapeMode::STOP_VISIBLE;
}

void DomeShape::turnOff(void) {
    mode = DomeShapeMode::DISAPPEAR;
}

int DomeShape::setPhi(double ph_i) {
    if (ph_i>=0.0 && ph_i<=PI) {
        ph = ph_i;
        return 0;
    }
    else return 1;
}

int DomeShape::setTheta(double th_i) {
    if (th_i>=0.0 && th_i<=2*PI) {
        th = th_i;
        return 0;
    }
    else return 1;
}

int DomeShape::setType(string type_i) {
    if (!type_i.empty()) {
        type = type_i;
        return 0;
    }
    else return 1;
}

int DomeShape::setName(string name_i) {
    if (!name_i.empty()) {
        name = name_i;
        return 0;
    }
    else return 1;
}
