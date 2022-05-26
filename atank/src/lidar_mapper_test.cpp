#include "lidar_mapper.h"

#include <iostream>
#include <thread>

LidarMapper lmapper;

void TEST_procRawLidarFrame() {
    std::ifstream ifs;

    ifs.open("lidarDump.dat", std::ios::binary | std::ios::in);

    if (!ifs.is_open()) {
        std::cerr << "lidar data dump file open fail!" << std::endl;
        return;
    }

    int test_count = 0;

    while (!ifs.eof()) {
        uint16_t size;
        uint8_t  buf[2048];
        ifs.read((char*)&size, sizeof(uint16_t));
        ifs.read((char*)buf, sizeof(uint8_t) * size);
        std::cout << "detect (raw)frame data[" << size << "]\n";

        std::cout << "data[0] = [" << (int)buf[0] << ", ";
        std::cout << (int)buf[1] << ", ";
        std::cout << (int)buf[2] << ", ";
        std::cout << (int)buf[3] << "\n";

#if 1
        // create vector typed buffer.
        std::vector<uint8_t> vbuf;
        for (int i = 0; i < size; i++) {
            vbuf.push_back(buf[i]);
        }

        // calls a test function 'procRawLidarFrame'.
        lmapper.procRawLidarFrame(vbuf);

        //if (test_count++ > 40) {
        //    break;
        //}
#endif
    }

    ifs.close();
}


int main(int argc, char *argv[])
{
    std::thread _callback(TEST_procRawLidarFrame);
    std::cout << "'callback' test thread is created!" << std::endl;

    std::thread _window(lmapper.initOpenGL, &lmapper, argc, argv);
    std::cout << "'opengl'   test thread is created!" << std::endl;

    _callback.join();
    _window.join();
    
    return 0;
}
