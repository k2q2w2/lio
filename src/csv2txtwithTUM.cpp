//
// Created by nepgear on 2024/2/28.
//
#include "iostream"
#include "fstream"
#include "string"
#include <sstream>
#include "opencv4/opencv2/opencv.hpp"

int main() {
    const std::string FILE_PATH_ = "/home/nepgear/dataset/gate_01.txt";
    const std::string des_FILE_PATH_ = "/home/nepgear/ws_livox/src/lio/log/gate_gt.txt";
    double origin_x, origin_y, origin_z;
    std::fstream file(FILE_PATH_), des_file(des_FILE_PATH_, std::ios::out);
    std::string buffer;
    int count = 1;
    cv::Mat src;
    while (std::getline(file, buffer)) {
        std::stringstream ssp(buffer);
        std::string new_buffer;
        if (count == 0) {
            count++;
            continue;
        } else {
            des_file << count;
            int cc = 0;

            while (std::getline(ssp, new_buffer, ' ')) {
                cc++;
                if (cc > 1) {
                    des_file << " " << new_buffer;
                }
            }
            des_file << std::endl;
            count++;
        }
//        csv file state;
//        double time, x, y, z, qx, qy, qz, qw;
//        ssp >> time >> x >> y >> z >> qx >> qy >> qz >> qw;
//        des_file << count << " " << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw
//                 << std::endl;
//        count++;
    }
    file.close();
    des_file.close();
}