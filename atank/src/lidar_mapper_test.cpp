#include "lidar_mapper.h"

#include <iostream>
#include <thread>

void TEST_procRawLidarFrame() {
    LidarMapper *lmapper = LidarMapper::GetInstance();

    std::ifstream ifs;
    std::string fname("lidarDump.dat");

    ifs.open(fname, std::ios::binary | std::ios::in);

    if (!ifs.is_open()) {
        std::cerr << "lidar data dump file open fail!" << std::endl;
        return;
    }

    std::cout << "Read data flie '" << fname << "'...." << std::endl;

    int test_count = 0;

    while (!ifs.eof()) {
        uint16_t size;
        uint8_t  buf[2048];
        ifs.read((char*)&size, sizeof(uint16_t));
        ifs.read((char*)buf, sizeof(uint8_t) * size);
        std::cout << "detect (raw)frame data[" << size << "]\n";

        //std::cout << "data[0] = [" << (int)buf[0] << ", ";
        //std::cout << (int)buf[1] << ", ";
        //std::cout << (int)buf[2] << ", ";
        //std::cout << (int)buf[3] << "\n";

        // create vector typed buffer.
        std::vector<uint8_t> vbuf;
        for (int i = 0; i < size; i++) {
            vbuf.push_back(buf[i]);
        }

        // calls a test function 'procRawLidarFrame'.
        lmapper->procRawLidarFrame(vbuf);
    }

    ifs.close();
}


int main(int argc, char *argv[])
{
    std::thread _callback(TEST_procRawLidarFrame);
    std::cout << "'callback' test thread is created!" << std::endl;

    std::thread _window(initOpenGL, argc, argv);
    std::cout << "'opengl'   test thread is created!" << std::endl;

    _callback.join();
    _window.join();
    
    return 0;
}
