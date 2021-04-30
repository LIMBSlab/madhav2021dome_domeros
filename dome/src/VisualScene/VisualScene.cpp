#include "VisualScene.h"
using namespace std;

#define PI 3.1415926535897932384626433832795

/***** GLOBALS *****/
auto start = chrono::steady_clock::now();
auto finish = chrono::steady_clock::now();

DomeShape* cue;
CueConstellation cues;
map<string, double> cueGroupAngle;
map<int, string> cueGroupRenderOrder;

double offset; // Brings the zero of the visual scene to the lab zero, done using the visual calibration jig and the projected grid with a line at zero longitude.

dome_common_msgs::DomeEvent event;
ros::Publisher event_pub;
ros::Publisher obj_pub;
ros::ServiceServer whats_in_dome_srv;

double timestamp;

// Shader params
// GLhandleARB v, p;

// Screen params
bool window_fullScreen = false;
bool displayOnly = false;

time_t time1=0;


/***** FUNCTIONS *****/
dome_common_msgs::DomeObj objectFromCue(DomeShape* cue, double groupTheta = 0.0) {
    dome_common_msgs::DomeObj domeObj;

    domeObj.type = cue->getType();
    domeObj.name = cue->getName();
    domeObj.theta = fmod(cue->getTheta() + groupTheta, 2*PI);
    domeObj.phi = cue->getPhi();
    domeObj.event = DomeShapeModeStrings[cue->getMode()];

    return domeObj;
}

void quitVisualScene(void) {
    ROS_INFO("Visual scene exiting");
    ros::shutdown();
    glutDestroyWindow(glutGetWindow());
    exit(0);
}


// Main display function
void mainDisplay(void) {
    time1=glutGet(GLUT_ELAPSED_TIME);

    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    /** \note{Rendering is back to front}*/
    DomeShape* cue;
    string cueGroupName;
    double groupTheta;
    for (auto it = cueGroupRenderOrder.begin(); it != cueGroupRenderOrder.end(); it++) {
        cueGroupName = it->second;

        CueGroup cueGroup = cues[cueGroupName];
        groupTheta = cueGroupAngle[cueGroupName] + offset;

        glRotatef(groupTheta, 0.0, 0.0, 1.0);
        for (auto it2 = cueGroup.begin(); it2 != cueGroup.end(); it2++) {
            cue = it2->second;
            if (cue->render(time1) && !displayOnly) obj_pub.publish(objectFromCue(cue, groupTheta));
        }
        glRotatef(-groupTheta, 0.0, 0.0, 1.0);
    }

    glutSwapBuffers();
    // glFlush();


    /// Compute time elapsed for current frame and publish it
    finish = chrono::steady_clock::now();
    // cerr << "Elapsed time in microseconds : "
    //         << chrono::duration_cast<chrono::microseconds>(finish - start).count()
    //         << " µs" << endl;

    if (!displayOnly) {
        // cerr << "Latency: " << (ros::WallTime::now().toNSec()-timestamp)/1.0e3 << "us" << endl;
        // cerr << "Time: " << setprecision(30) << static_cast<float>(ros::WallTime::now().toNSec()) << endl;
        // event.value = chrono::duration_cast<chrono::microseconds>(finish - start).count();
        event.value = (ros::Time::now().toNSec()-timestamp)/1.0e3;
        event_pub.publish(event);
    }
    start = chrono::steady_clock::now();

    /// Spin ROS
    ros::spinOnce();
}

// Vertex shader is compiled and attached to program through this function
void setShaders(void) {
    // Read our shaders into the appropriate buffers
/*
    ifstream ifs_vert("mirrorProg.vert");
    string vertexSource( (istreambuf_iterator<char>(ifs_vert) ),
                       (istreambuf_iterator<char>()    ) );
    ROS_INFO_STREAM(vertexSource);

    ifstream ifs_frag("simpleFragShader.frag");
    string fragmentSource( (istreambuf_iterator<char>(ifs_frag) ),
                       (istreambuf_iterator<char>()    ) );
    ROS_INFO_STREAM(fragmentSource);
*/
    string vertexString = R"(void main() {
        vec3 zvec = vec3(0.0,0.0,1.0);
        vec3 posvec = normalize(vec3(gl_Vertex));
        float sinp = abs(dot(zvec,posvec));

        float rat = sqrt(1.0/(1.0+1.0*sinp));

        gl_Position = ftransform();
        gl_Position.x *= rat;
        gl_Position.y *= rat;
	})";

    string fragmentString = R"(void main() {
    	gl_FragColor = gl_Color;
	})";

    // Create an empty vertex shader handle
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);

    // Send the vertex shader source code to GL
    // Note that string's .c_str is NULL character terminated.
    const GLchar *vertexSource = vertexString.c_str();
    glShaderSource(vertexShader, 1, &vertexSource, NULL);

    // Compile the vertex shader
    glCompileShader(vertexShader);

    GLint isCompiled = 0;
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &isCompiled);
    if(isCompiled == GL_FALSE)
    {
        cerr << "Vertex shader not compiled";

	    GLint maxLength = 0;
	    glGetShaderiv(vertexShader, GL_INFO_LOG_LENGTH, &maxLength);

	    // The maxLength includes the NULL character
	    vector<GLchar> infoLog(maxLength);
	    glGetShaderInfoLog(vertexShader, maxLength, &maxLength, &infoLog[0]);
	    string log(infoLog.begin(),infoLog.end());
        cerr << log;

	    // We don't need the shader anymore.
	    glDeleteShader(vertexShader);
	    return;
    }

    // Create an empty fragment shader handle
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    // Send the fragment shader source code to GL
    // Note that string's .c_str is NULL character terminated.
    const GLchar *fragmentSource = fragmentString.c_str();
    glShaderSource(fragmentShader, 1, &fragmentSource, NULL);

    // Compile the fragment shader
    glCompileShader(fragmentShader);

    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &isCompiled);
    if (isCompiled == GL_FALSE)
    {
        cerr << "Fragment shader not compiled";

	    GLint maxLength = 0;
	    glGetShaderiv(fragmentShader, GL_INFO_LOG_LENGTH, &maxLength);

	    // The maxLength includes the NULL character
	    vector<GLchar> infoLog(maxLength);
	    glGetShaderInfoLog(fragmentShader, maxLength, &maxLength, &infoLog[0]);
	    
	    string log(infoLog.begin(),infoLog.end());
        cerr << log;

	    // We don't need the shader anymore.
	    glDeleteShader(fragmentShader);
	    // Either of them. Don't leak shaders.
	    glDeleteShader(vertexShader);

	    return;
    }

    GLuint program = glCreateProgram();

    // Attach our shaders to our program
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);

    // Link our program
    glLinkProgram(program);

    // Note the different functions here: glGetProgram* instead of glGetShader*.
    GLint isLinked = 0;
    glGetProgramiv(program, GL_LINK_STATUS, (int *)&isLinked);
    if (isLinked == GL_FALSE)
    {
        cerr << "Shaders not linked";

	    GLint maxLength = 0;
	    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &maxLength);

	    // The maxLength includes the NULL character
	    vector<GLchar> infoLog(maxLength);
	    glGetProgramInfoLog(program, maxLength, &maxLength, &infoLog[0]);

	    string log(infoLog.begin(),infoLog.end());
        cerr << log;

	    // We don't need the program anymore.
	    glDeleteProgram(program);
	    // Don't leak shaders either.
	    glDeleteShader(vertexShader);
	    glDeleteShader(fragmentShader);

	    return;
    }

    glUseProgram(program);

    // Always detach shaders after a successful link.
    // glDetachShader(program, vertexShader);
}

void landmark_angle_callback(const std_msgs::Float64MultiArray::ConstPtr& ang) {
    // This is hacky. The right way to do this would be for the message to have a cue group field and then set that angle. 
    // Good enough for now, but will throw an error if the yaml file does not have "landmarks" and "stripes" cue groups
    cueGroupAngle["landmarks"] = ang->data[0];
    cueGroupAngle["stripes"] = ang->data[0];
    timestamp = ang->data[1];
    //timestamp = ros::Time::now().toNSec();
}

void quit_callback(const dome_common_msgs::DomeEvent::ConstPtr& quit) {
    quitVisualScene();
}

void visible_callback(const dome_common_msgs::DomeVisible::ConstPtr& vis) {
    auto it = cues.find(vis->type);
    if (it != cues.end()) {
        CueGroup cueGroup = it->second;

        for (auto it2 = cueGroup.begin(); it2 != cueGroup.end(); it2++) {
            it2->second->fadeOnOff(vis->visible);
        }
    }
}

void makeCues(CueConstellationParamMap &cueConstellationParamMap) {
    string cueGroupName, cueName, shapeName;
    CueGroupParamMap cueGroupParamMap;
    CueParamMap cueParamMap;
    DomeShape* shape;
    int numRepeats;
    int defaultRenderOrder = INT_MIN;

    for (auto it = cueConstellationParamMap.begin(); it != cueConstellationParamMap.end(); it++) {
        cueGroupName = it->first;
        cueGroupParamMap = it->second;

        cueGroupAngle[cueGroupName] = 0.0;
        CueGroup cueGroup;

        bool hasRenderOrder = false;

        for (auto it2 = cueGroupParamMap.begin(); it2 != cueGroupParamMap.end(); it2++) {
            cueName = it2->first;
            cueParamMap = it2->second;

            if (!cueName.compare("_group_params")) {
                auto it3 = cueParamMap.find("theta");
                if (it3 != cueParamMap.end()) cueGroupAngle[cueGroupName] = it3->second;

                it3 = cueParamMap.find("render_order");
                if (it3 != cueParamMap.end()) {
                    cueGroupRenderOrder[it3->second] = cueGroupName;
                    hasRenderOrder = true;
                }
            }
            else {
                auto it3 = cueParamMap.find("repeat");
                if (it3 != cueParamMap.end()) numRepeats = it3->second;
                else numRepeats = 1;

                for (int r = 1; r<=numRepeats; r++) {
                    it3 = cueParamMap.find("shape");
                    if (it3 == cueParamMap.end()) shape = DomeShapeLibrary::Shape("", time1);
                    else shape = DomeShapeLibrary::Shape(it3->second, time1);

                    if (numRepeats>1) shapeName = cueName + to_string(r);
                    else shapeName = cueName;
                    shape->setName(shapeName);

                    shape->setType(cueGroupName);

                    it3 = cueParamMap.find("theta");
                    if (it3 != cueParamMap.end()) shape->setTheta(double(it3->second) * PI / 180);

                    it3 = cueParamMap.find("phi");
                    if (it3 != cueParamMap.end()) shape->setPhi(double(it3->second) * PI / 180);

                    it3 = cueParamMap.find("fade_in_ms");
                    if (it3 != cueParamMap.end()) shape->setFadeIn(it3->second);

                    it3 = cueParamMap.find("fade_out_ms");
                    if (it3 != cueParamMap.end()) shape->setFadeOut(it3->second);

                    it3 = cueParamMap.find("visible");
                    if (it3 != cueParamMap.end()) shape->turnOnOff(it3->second);

                    it3 = cueParamMap.find("cycle_probability");
                    if (it3 != cueParamMap.end()) shape->setCycleProbability(it3->second);

                    shape->makeList();
                    cueGroup[shapeName] = shape;
                }
            }
        }

        cues[cueGroupName] = cueGroup;
        if (!hasRenderOrder) {
            cueGroupRenderOrder[defaultRenderOrder] = cueGroupName;
            defaultRenderOrder++;
        }
    }
}

void toggleFullscreen(void)
{
    if (window_fullScreen) {
        glutReshapeWindow(glutGet(GLUT_INIT_WINDOW_WIDTH), glutGet(GLUT_INIT_WINDOW_HEIGHT));
        glutPositionWindow(glutGet(GLUT_INIT_WINDOW_X), glutGet(GLUT_INIT_WINDOW_Y));
        window_fullScreen = false;
    }
    else {
        glutFullScreen();
        window_fullScreen = true;
    }
}

/** \name whatsInDome
 * Service that returns a list of dome objects that are visible
 */
bool whatsInDome(dome_common_msgs::WhatsInDome::Request &req, dome_common_msgs::WhatsInDome::Response &res)
{
    DomeShape* cue;
    double groupTheta;
    for (auto it = cues.begin(); it != cues.end(); it++) {
    	if (!req.type.compare(it->first) || !req.type.compare("")) {
	        CueGroup cueGroup = it->second;

	        groupTheta = cueGroupAngle[it->first] + offset;
	        for (auto it2 = cueGroup.begin(); it2 != cueGroup.end(); it2++) {
	            cue = it2->second;
	            if (cue->getMode() != DomeShapeMode::OFF) res.objects.push_back(objectFromCue(cue, groupTheta));
	        }
	    }
    }
    
    return true;
}

/// Screen reshape function (maintains aspect ratio when reshaping screen)
void reshape(int w, int h) {
    if(h == 0) h = 1;
    GLdouble aspect = (GLdouble)w / (GLdouble)h;
    glMatrixMode   (GL_PROJECTION);
    glLoadIdentity ();
    glOrtho(-DomeShape::R*aspect, DomeShape::R*aspect, -DomeShape::R, DomeShape::R, 0.0, DomeShape::R);
    glMatrixMode   (GL_MODELVIEW);
    glViewport (0, 0, w, h);
}

// Key press function
void keyPressed (unsigned char key, int x, int y) {  
    switch (key) {
        case 'q':
	        quitVisualScene();
            break;
        case 'p':
            toggleFullscreen();
            break;
        default:
            ;
    }
}  


// Main loop
int main(int argc, char** argv) {
    ros::init(argc, argv, "VisualScene",ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::NodeHandle nh("~");


    // Parse cues in parameter server - convert from an xml parameter tree to a nested map
    XmlRpcValue cueConstellationParamTree;
    nh.getParam("/cues",cueConstellationParamTree);
    ROS_ASSERT(cueConstellationParamTree.getType() == XmlRpcValue::TypeStruct);
    CueConstellationParamMap cueConstellationParamMap;
    for (auto it = cueConstellationParamTree.begin(); it != cueConstellationParamTree.end(); ++it) {
        XmlRpcValue cueGroupParamTree = it->second;
        ROS_ASSERT(cueGroupParamTree.getType() == XmlRpcValue::TypeStruct);

        CueGroupParamMap cueGroupParamMap;
        for (auto it2 = cueGroupParamTree.begin(); it2 != cueGroupParamTree.end(); ++it2) {
            XmlRpcValue cueParamTree = it2->second;
            ROS_ASSERT(cueParamTree.getType() == XmlRpcValue::TypeStruct);

            CueParamMap cueParamMap;
            for (auto it3 = cueParamTree.begin(); it3 != cueParamTree.end(); ++it3) {
                cueParamMap[it3->first] = it3->second;
            }
            cueGroupParamMap[it2->first] = cueParamMap;
        }
        cueConstellationParamMap[it->first] = cueGroupParamMap;
    }

    //* Parameter to determine if the node is run as display only. If true, the node would not publish or offer services.
    int window_posX, window_posY, window_width, window_height;
    nh.param<bool>("displayOnly", displayOnly, false);
    nh.param<int>("posX", window_posX, 10);
    nh.param<int>("posY", window_posY, 10);
    nh.param<int>("width", window_width, 800);
    nh.param<int>("height", window_height, 900);
    nh.param<double>("offset",offset, -61.5);

    // Subscribers
    ros::Subscriber landmark_angle_sub = n.subscribe("landmark_angle", 1000, landmark_angle_callback);
    ros::Subscriber visible_sub = n.subscribe("dome_visible", 1000, visible_callback);
    ros::Subscriber quit_sub = n.subscribe("quit",1,quit_callback);
	
    // Publishers
	if (!displayOnly) {
		event.name = "refresh";
		event_pub = n.advertise<dome_common_msgs::DomeEvent>("dome_event", 1000);
		ros::spinOnce();

		obj_pub = n.advertise<dome_common_msgs::DomeObj>("dome_objects", 1000);
		ros::spinOnce();

	    whats_in_dome_srv = n.advertiseService("whats_in_dome",whatsInDome);
		ros::spinOnce();
	}

    srand(time(NULL));

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA | GLUT_MULTISAMPLE); //|GLUT_ACCUM);
    glutInitWindowPosition(window_posX, window_posY);
    glutInitWindowSize(window_width, window_height);

    glutCreateWindow("Dome Visual Scene");

    //glewInit();
    //glXSwapIntervalSGI(1);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    /// Smoothing without multisampling
    // http://bankslab.berkeley.edu/members/chris/AntiAliasing/AntiAliasingInOpenGL.html
    // glEnable(GL_LINE_SMOOTH);
    // glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    // glEnable(GL_POINT_SMOOTH);
    // glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    // glDisable(GL_MULTISAMPLE);

    /// Smoothing with multisampling
    glEnable(GL_MULTISAMPLE);
    glHint(GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);
    
    glClearColor(0.0,0.0,0.0,0.0);

    time1=glutGet(GLUT_ELAPSED_TIME);

    // Initialize and load cues
    makeCues(cueConstellationParamMap);


    // Display functions
    glutDisplayFunc(mainDisplay); 
    glutIdleFunc(glutPostRedisplay);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyPressed);

    // glewInit();
    // setShaders();

    glutMainLoop();

    return 0;
}
