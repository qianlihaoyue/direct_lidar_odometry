/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/
#pragma once

#include "type.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nano_gicp/nano_gicp.hpp>

#include <chrono>
#include <iostream>

class Timer {
public:
    // 初始化的时候会记载一次开始时间，const变量只能在构造函数的初始化列表中进行初始化
    Timer() { tic(); }
    Timer(const std::string& nameIn) : name(nameIn) { tic(); }

    // 重置开始时间
    void tic() { start = std::chrono::system_clock::now(); }

    // 计算结束时间
    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> dt = end - start;
        start = end;
        return dt.count() * 1000;
    }

    // 含有输出的结束
    void toc_cout() { std::cout << "[" << name << "]:" << toc() << "ms" << std::endl; }

private:
    const std::string name;
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

#define TIMER_CREATE(name) Timer name(#name)