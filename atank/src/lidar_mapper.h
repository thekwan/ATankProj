#include <fstream>
#include <vector>
#include <list>
#include <mutex>

/* OpenGL function includes
 */
#include <GL/glut.h>

typedef struct LidarPacket_ {
    uint8_t qual;   // quality
    float distance; // distance
    float angle;    // angle(degree)
} LidarPacket;

typedef struct LidarFrame_ {
    std::vector<LidarPacket> packets;
} LidarFrame;

class LidarMapper {
public:
    static LidarMapper *GetInstance(void);
    LidarMapper(int max_frame_size);
    ~LidarMapper();
    void procRawLidarFrame(std::vector<uint8_t> data);
    void dumpRawByte(std::vector<uint8_t> data);
    LidarFrame *getLastLidarFrame(void);
    LidarFrame *getLidarFrame(int index);

    // Test functions
    static void TEST_procRawLidarFrame(void);
private:
    float getAngleDegree(uint8_t high, uint8_t low);
    float getSpeedHz(uint8_t high, uint8_t low);
    void addLidarPacket(LidarPacket &packet);
    void printSuperFrame(LidarFrame &lframe);
    void addLidarFrame(LidarFrame &lframe);
    void writeFileLidarFrame(LidarFrame &lframe);

    float lastPacket_angle_;
    const int maxSuperFrameSize;

    std::mutex  mutex_superFrames_;

    // ONLY FOR DEBUG
    std::ofstream  rawbyteFilePtr_;
    std::ofstream  sframeFilePtr_;

    std::vector<uint8_t> oldbytes_;
    std::vector<LidarPacket> rawPackets_;
    std::list<LidarFrame> superFrames_;
};

/* OpenGL GLUT related functions.
 */
void initGL();
void initOpenGL(int argc, char *argv[]);
void doKeyboard(unsigned char key, int x, int y);
void doSpecial(int key, int x, int y);
void reshape(GLsizei width, GLsizei height);
void display();

extern float point_scale;
extern float point_pos_x;
extern float point_pos_y;
extern int   map_index;
