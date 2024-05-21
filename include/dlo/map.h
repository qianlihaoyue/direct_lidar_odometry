/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/dlo.h"
#include <pcl/io/pcd_io.h>
// #include <direct_lidar_odometry/srv/save_pcd.>

class MapNode : public rclcpp::Node {
public:
    MapNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    template <typename T>
    void declare_and_get_parameter(const std::string& param_yaml, const T& default_value, T& param_var) {
        this->declare_parameter<T>(param_yaml, default_value);
        this->get_parameter(param_yaml, param_var);
    }

    void publishTimerCB();
    void keyframeCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr keyframe);

    rclcpp::TimerBase::SharedPtr publish_timer;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;

    // rclcpp::Service<direct_lidar_odometry::srv::SavePcd>::SharedPtr save_pcd_srv_;
    // void savePcd(const std::shared_ptr<direct_lidar_odometry::srv::SavePcd::Request> request,
    //              std::shared_ptr<direct_lidar_odometry::srv::SavePcd::Response> response);

    CloudPtr dlo_map;
    pcl::VoxelGrid<PointType> voxelgrid;

    rclcpp::Time map_stamp;
    std::string odom_frame;

    bool publish_full_map_;
    double publish_freq_;
    double leaf_size_;
};
