//
// Created by nepgear on 2024/3/4.
//
#pragma once

#include "common.h"

class Timer {
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
            start_{}, end_{};
    std::map<std::string, std::vector<double>> time_log_;

    std::string FLAG_log_dir_ = "/home/nepgear/ws_livox/src/lio/log/";
    std::string file_name_ = "record.txt";
    bool default_print_log_ = true;

    //name,{count,elapsed_time};
    void timer(std::string name, std::function<void(void)> this_func);

    void print_log();

    Timer(char **argv);

    Timer() = default;

    ~Timer();
};