#include "lidar_mapper.h"
//#include "ros/ros.h"

#include <iostream>
#include <algorithm>

LidarMapper::LidarMapper() {
}

LidarMapper::~LidarMapper() {
    if (!rawbyteFilePtr_.is_open()) {
        rawbyteFilePtr_.close();
    }
}

void LidarMapper::dumpRawByte(std::vector<uint8_t> data) {
    if (rawbyteFilePtr_.is_open()) {
        // write header(data size) 16bits.
        uint16_t size = (uint16_t) data.size();
        rawbyteFilePtr_.write((char*)&size, sizeof(uint16_t));

        // write byte datas
        for (auto &a : data) {
            rawbyteFilePtr_.write((char*)&a, sizeof(uint8_t));
        }
    }
    else {
        rawbyteFilePtr_.open("lidarDump.dat", std::ios::binary | std::ios::out);
    }
}

float LidarMapper::getAngleDegree(uint8_t high, uint8_t low) {
    float angle = ((float)high * 256.0 + (float)low) / 64.0 - 640.0;
    angle += (angle < 0) ? 360.0 : 0;

    return angle;
}

float LidarMapper::getSpeedHz(uint8_t high, uint8_t low) {
    return ((float)high*256.0 + (float)low) / 3840.0;
}

void LidarMapper::procRawLidarFrame(std::vector<uint8_t> bytes) {
    std::vector<uint8_t> header = {0x55, 0xAA, 0x03, 0x08};

    //ROS_INFO("procRawLidarFrame()");

    // concatenate old and new data.
    oldbytes_.insert( oldbytes_.end(), bytes.begin(), bytes.end() );

    // find byte stream header (0x55, 0xAA, 0x03, 0x08) and 
    // generates Lidar packets.
    auto it_start  = oldbytes_.begin();

    for (int i = 0; i < oldbytes_.size(); i++) {
        auto it = std::search(it_start, oldbytes_.end(),
                header.begin(), header.end());

        if (it != oldbytes_.end() && std::distance(it, oldbytes_.end()) > 34) {
            // FOUND
            //std::cout << "FOUND lidar frame" << std::endl;
            it += 4;
            
            // speed
            uint8_t byteL, byteH;
            byteL = *it++; byteH = *it++;
            float speed = getSpeedHz(byteH, byteL);

            // angle 
            float angleS, angleE;
            byteL = *it++; byteH = *it++;
            angleS = getAngleDegree(byteH, byteL);
            byteL = *(it+24); byteH = *(it+25);
            angleE = getAngleDegree(byteH, byteL);

            float angle_step = 0;
            if (angleE > angleS) {
                angle_step = (angleE - angleS) / 8.0;
            }
            else if (angleS > angleE) {
                angle_step = (angleE - (angleS - 360.0)) / 8.0;
            }

            if (angle_step == 0) {
                //ROS_INFO("INVALID RAW FRAME: skip this.");
                it_start = it;
                continue;
            }

            // distance and range data
            lidarPacket  pk;
            for (int i = 0; i < 8; i++) {
                byteL = *it++;
                byteH = *it++;
                pk.distance = (float)(byteH)*256.0 + (float)(byteL);
                pk.qual = *it++;
                pk.angle = angleS + angle_step * i;
                //rawPackets_.push_back(pk);
            }
            it++;   // skip 2 bytes for 'end_angle'
            it++;

            //ROS_INFO("angle(s,e) = %3.2f , %3.2f  step(%1.2f)", 
            //        angleS, angleE, angle_step);
            
            // remove processed bytes
            it_start = it;// + 30;
        }
        else {
            // NOT FOUND
            //ROS_INFO("NOT FOUND lidar frame");
            if (it_start == oldbytes_.end()) {
                oldbytes_.clear();
            }
            else {
                int a = std::distance(it_start, oldbytes_.end());
                oldbytes_.erase(oldbytes_.begin(), it_start);
            }
            break;
        }
    }

    // find a start position of 36bytes data frame.
    //
    // get several raw data frames (36bytes)
    //
    // create super-frame
    //
    // wakes map generator if there is new super-frame.
}


static float point_scale;
static float point_pos_x;
static float point_pos_y;
static int   map_index;

void LidarMapper::initGL() {
    // set "clearing" or background color
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);   // black and opaque
}

void LidarMapper::display() {
    glClear(GL_COLOR_BUFFER_BIT);

    //std::vector<point2_t> pts = mapmng.get_map_point(map_index);
    //std::vector<point2_t> pts;

    // Define shapes enclosed within a pair of glBegin and glEnd
    glBegin(GL_POINTS);
        glColor3f(1.0f, 1.0f, 0.0f);    // Red
        //for(auto &a : pts) {
        //    glVertex2f(point_pos_x + a.x * point_scale, point_pos_y + a.y * point_scale);
        //}
    glEnd();

    glFlush();
}

void LidarMapper::reshape(GLsizei width, GLsizei height) {
    // Compute aspect ratio of the new windows
    if (height == 0) height = 1;
    GLfloat aspect = (GLfloat) width / (GLfloat) height;

    // Set the viewport to cover the new window.
    glViewport(0, 0, width, height);

    // Set the aspect ratio of the clipping area to match the Viewport
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (width >= height) {
        // aspect >= 1, set the height from -1 to 1, with larger width
        gluOrtho2D(-1.0 * aspect, 1.0 * aspect, -1.0, 1.0);
    }
    else {
        // aspect < 1, set the width to -1 to 1, with larger height
        gluOrtho2D(-1.0, 1.0, -1.0 / aspect, 1.0 / aspect);
    }
}

void LidarMapper::doSpecial(int key, int x, int y) {
    switch(key) {
        case GLUT_KEY_LEFT: 
            point_pos_x -= 0.01;
            break;
        case GLUT_KEY_RIGHT:
            point_pos_x += 0.01;
            break;
        case GLUT_KEY_UP:
            point_pos_y += 0.01;
            break;

        case GLUT_KEY_DOWN:
            point_pos_y -= 0.01;
            break;
    }
    glutPostRedisplay();
}

void LidarMapper::doKeyboard(unsigned char key, int x, int y) {
    switch(key) {
        case 'x':
        case 'X':
            point_scale *= 1.1;
            break;
        case 'z':
        case 'Z':
            point_scale /= 1.1;
            break;
        case 'q':
        case 'Q':
            exit(0);
            break;
        case 'i':
        case 'I':
            //int max_index = mapmng.get_map_num();
            int max_index = 0;
            map_index++;
            if (map_index >= max_index) {
                map_index = max_index-1;
            }
            break;
    }
    glutPostRedisplay();
}

void LidarMapper::initOpenGL(LidarMapper *lm, int argc, char *argv[]) {
    point_scale = 1/16384.0;
    point_pos_x = 0.0;
    point_pos_y = 0.0;
    map_index = 0;

    glutInit(&argc, argv);
    glutInitWindowSize(640,480);
    glutInitWindowPosition(50,50);
    glutCreateWindow("Viewport Transform");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(doKeyboard);
    glutSpecialFunc(doSpecial);
    lm->initGL();
    glutMainLoop();
    return;
}
