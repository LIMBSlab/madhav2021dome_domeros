# pragma once
#include <time.h>
#include <string>
#include <vector>
#include <map>
#include "GL/gl.h"
#include <GL/glu.h>

using namespace std;

enum class DomeShapeMode {
        DISAPPEAR,
        OFF,
        START_VISIBLE,
        FADE_IN,
        CYCLE_IN,
        APPEAR,
        ON,
        STOP_VISIBLE,
        FADE_OUT,
        CYCLE_OUT,
        CYCLE_START
    };

map<DomeShapeMode, string> DomeShapeModeStrings = {
    {DomeShapeMode::DISAPPEAR, "Disappear"},
    {DomeShapeMode::OFF, "Off"},
    {DomeShapeMode::START_VISIBLE, "Start visible"},
    {DomeShapeMode::FADE_IN, "Fade in"},
    {DomeShapeMode::CYCLE_IN, "Cycle in"},
    {DomeShapeMode::APPEAR, "Appear"},
    {DomeShapeMode::ON, "On"},
    {DomeShapeMode::STOP_VISIBLE, "Stop visible"},
    {DomeShapeMode::FADE_OUT, "Fade out"},
    {DomeShapeMode::CYCLE_OUT, "Cycle out"},
    {DomeShapeMode::CYCLE_START, "Cycle start"}
};

class DomeShape {
    public:
        static const int R = 220;               //< Dome radius

        // Constructors
        DomeShape(time_t);
        DomeShape(time_t, double, double);
            
        // Getters
        double getTheta() const {return th;}
        double getPhi() const {return ph;}
        string getName() const {return name;}
        string getType() const {return type;}
        DomeShapeMode getMode() const {return mode;}

        // Setters
        int setAlphaDisplayList(GLuint);
        int setBlackDisplayList(GLuint);
        int setColor(double, double, double);
        int setColor(double);
        void setMode(DomeShapeMode);
        int setPhi(double);
        int setTheta(double);
        int setType(string);
        int setName(string);
        int setFadeIn(double);
        int setFadeOut(double);
        int setCycleProbability(double);

        // Member functions
        void fadeOnOff(bool);
        void turnOnOff(bool);
        void moveToDome(void);
        bool render(time_t);
        void fadeOff(void);
        void fadeOn(void);
        void turnOff(void);
        void turnOn(void);
        void cycle(void);
        void randomizePosition(void);
        virtual void makeList(void) = 0;

    protected:
        DomeShapeMode mode;                     //< Display mode of shape
        double color[3];                        //< Color of shape
        int fadeInTime;                         //< Fade in time (ms)
        int fadeOutTime;                        //< Fade out time (ms)
        double cycleProbability;
        double th;                              //< azimuth
        double ph;                              //< elevation
        string type;
        string name;
        time_t birthTime, doomTime;
        GLuint dlIdxAlpha, dlIdxBlack;          //< There are two OpenGL display lists per shape
        bool hasAlphaList, hasBlackList;

    private:
        // static int numberOfObjects;
};
