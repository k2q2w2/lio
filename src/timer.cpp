//
// Created by nepgear on 2024/3/4.
//
#include "timer.h"

Timer::Timer(char **argv) {
//    ros::param::get("log_dir", FLAG_log_dir_);
//    ros::param::get("default_print_log", default_print_log_);
//    FLAGS_log_dir = FLAG_log_dir_;
//    google::InitGoogleLogging(*argv);
}

void Timer::timer(std::string name, std::function<void()> this_func) {
    auto start = std::chrono::system_clock::now();
    this_func();
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>
            (end - start);
    if (time_log_.find(name) != time_log_.end()) {
        time_log_[name].push_back(elapsedTime.count());
    } else {
        time_log_.insert(std::make_pair(name, std::vector<double>{elapsedTime.count()}));
    }
}

Timer::~Timer() {
    print_log();
}

void Timer::print_log() {
    if (!default_print_log_) {
        return;
    } else {
        //写入时间文件，计算平均值
        std::ofstream file(FLAG_log_dir_ + file_name_, std::ios::ate);
        if (file.is_open()) std::cout << "success";
        double sum, count;
        for (auto i: time_log_) {
            sum = std::accumulate(i.second.begin(), i.second.end(), 0.0f);
            count = i.second.size();
            auto [min, max] = std::minmax_element(i.second.begin(), i.second.end());
            double avr = sum / count;
            file << std::setprecision(10) << i.first << " " << "avr:" << avr << " min & max:" << *min << " " << *max
                 << " " << "size"
                 << i.second.size() << std::endl;
        }
        file.close();
        return;
    }
}


