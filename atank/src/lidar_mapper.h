#include <fstream>
#include <vector>
#include <queue>

/* OpenGL function includes
 */
#include <GL/glut.h>

typedef struct lidarPacket_ {
    uint8_t qual;   // quality
    float distance; // distance
    float angle;    // angle(degree)
} lidarPacket;

typedef struct lidarFrame_ {
    std::vector<lidarPacket> packets;
} lidarFrame;

class LidarMapper {
public:
    LidarMapper();
    ~LidarMapper();
    void procRawLidarFrame(std::vector<uint8_t> data);
    void dumpRawByte(std::vector<uint8_t> data);

    // Test functions
    void TEST_procRawLidarFrame(void);
private:
    float getAngleDegree(uint8_t high, uint8_t low);
    float getSpeedHz(uint8_t high, uint8_t low);

    std::ofstream  rawbyteFilePtr_;
    std::vector<uint8_t> oldbytes_;
    std::queue<lidarPacket>  rawPackets_;
    std::vector<lidarFrame>  superFrames_;
};

class MapWindow {
public:
    MapWindow() {}
    ~MapWindow() {}
private:
    void initOpenGL(void);
    void doKeyboard(unsigned char key, int x, int y);
    void doSpecial(int key, int x, int y);
    void reshape(GLsizei width, GLsizei height);
    void display();
    void initGL();
};
