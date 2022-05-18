#include <fstream>
#include <vector>
#include <queue>

typedef struct lidarPacket_ {
    uint8_t qual;   // quality
    float dist;     // distance
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
private:
    bool rawByteFileOpened_;
    std::ofstream  rawbyteFilePtr_;
    std::vector<uint8_t> oldbytes_;
    std::queue<lidarPacket>  rawPackets_;
    std::vector<lidarFrame>  superFrames_;
};
