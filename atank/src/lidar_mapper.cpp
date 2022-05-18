#include "ros/ros.h"
#include "std_msgs/String.h"
#include "atank/LidarFrame.h"
#include "lidar_mapper.h"

#include <algorithm>
#include <thread>
#include <signal.h>

LidarMapper lmapper;

void mySignalHandler(int sig) {
    ROS_INFO("Signal handler is called. Server will be terminated.");
    ros::shutdown();
    exit(1);
}

void lidar_frame_callback(const atank::LidarFrame &msg) {
    ROS_INFO("LidarFrame: frame [%3d]", msg.size);
    
    // dump lidar raw byte date into a file.
    lmapper.dumpRawByte(msg.data);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_mapper");
    ros::NodeHandle n;

    signal(SIGINT, mySignalHandler);

    // "command": topic name
    ros::Subscriber sub = n.subscribe("lidar_frame", 1000, lidar_frame_callback);
    ROS_INFO("[CMD SERVER] 'subscriber' is ready to process.");

    ros::spin();

    return 0;
}


//////////////////////////////////////////////////////////
//
// LidarMapper class member functions
//
//////////////////////////////////////////////////////////
//
LidarMapper::LidarMapper() {
    rawByteFileOpened_ = false;
}

LidarMapper::~LidarMapper() {
    if (!rawByteFileOpened_) {
        rawbyteFilePtr_.close();
    }
}

void LidarMapper::dumpRawByte(std::vector<uint8_t> data) {
    if (!rawByteFileOpened_) {
        rawbyteFilePtr_.open("lidarDump.dat", std::ios::binary | std::ios::out);
        rawByteFileOpened_ = true;
    }

    if (rawByteFileOpened_) {
        for (auto &a : data) {
            rawbyteFilePtr_.write((char*)&a, sizeof(uint8_t));
        }
    }
}

void LidarMapper::procRawLidarFrame(std::vector<uint8_t> bytes) {
    std::vector<uint8_t> header = {0x55, 0xAA, 0x03, 0x08};

    // concatenate old and new data.
    oldbytes_.insert( oldbytes_.end(), bytes.begin(), bytes.end() );

    // find byte stream header (0x55, 0xAA, 0x03, 0x08) and 
    // generates Lidar packets.
    auto it_start  = oldbytes_.begin();

    for (int i = 0; i < oldbytes_.size(); i++) {
        auto it = std::search(it_start, oldbytes_.end(),
                header.begin(), header.end());

        if (it != oldbytes_.end() && std::distance(it, oldbytes_.end()) < 30) {
            // FOUND
            // speed bytes
            uint8_t speedByte[2], angleByte[2];
            speedByte[0] = *it++;
            speedByte[1] = *it++;
            angleByte[0] = *it++;
            angleByte[1] = *it++;

            for (int i = 0; i < 8; i++) {
            }
            // create lidar packets
            // remove processed bytes
            it_start = it + 30;
        }
        else {
            // NOT FOUND
            oldbytes_.erase(oldbytes_.begin(), it_start);
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
