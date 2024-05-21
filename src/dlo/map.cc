#include "dlo/map.h"

MapNode::MapNode(const rclcpp::NodeOptions& options) : Node("dlo_map_node", options) {
    declare_and_get_parameter<std::string>("odomNode.odom_frame", "odom", odom_frame);
    declare_and_get_parameter<bool>("mapNode.publishFullMap", true, publish_full_map_);
    declare_and_get_parameter<double>("mapNode.publishFreq", 1.0, publish_freq_);
    declare_and_get_parameter<double>("mapNode.leafSize", 0.5, leaf_size_);

    if (publish_full_map_)
        publish_timer = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / publish_freq_)), std::bind(&MapNode::publishTimerCB, this));
    keyframe_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("keyframe", rclcpp::SensorDataQoS(),
                                                                            std::bind(&MapNode::keyframeCB, this, std::placeholders::_1));
    map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 1);
    // save_pcd_srv_ =
    //     this->create_service<direct_lidar_odometry::srv::SavePcd>("SavePcd", std::bind(&MapNode::savePcd, this, std::placeholders::_1,
    //     std::placeholders::_2));

    dlo_map = CloudPtr(new CloudType);
    RCLCPP_INFO(rclcpp::get_logger("DirectLidarOdometry"), "DLO Map Node Initialized");
}

void MapNode::publishTimerCB() {
    if (dlo_map->points.size()) {
        sensor_msgs::msg::PointCloud2 map_ros;
        pcl::toROSMsg(*dlo_map, map_ros);
        map_ros.header.stamp = this->get_clock()->now();
        map_ros.header.frame_id = odom_frame;
        map_pub->publish(map_ros);
    }
}

void MapNode::keyframeCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr keyframe) {
    // convert scan to pcl format
    CloudPtr keyframe_pcl = CloudPtr(new CloudType);
    pcl::fromROSMsg(*keyframe, *keyframe_pcl);

    // voxel filter
    voxelgrid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxelgrid.setInputCloud(keyframe_pcl);
    voxelgrid.filter(*keyframe_pcl);

    // save keyframe to map
    map_stamp = keyframe->header.stamp;
    *dlo_map += *keyframe_pcl;

    if (!publish_full_map_) {
        if (keyframe_pcl->points.size() == keyframe_pcl->width * keyframe_pcl->height) {
            sensor_msgs::msg::PointCloud2 map_ros;
            pcl::toROSMsg(*keyframe_pcl, map_ros);
            map_ros.header.stamp = this->get_clock()->now();
            map_ros.header.frame_id = odom_frame;
            map_pub->publish(map_ros);
        }
    }
}

// void MapNode::savePcd(const std::shared_ptr<direct_lidar_odometry::srv::SavePcd::Request> request,
//                       std::shared_ptr<direct_lidar_odometry::srv::SavePcd::Response> response) {
//     CloudPtr m = CloudPtr(new CloudType(*dlo_map));  // Assuming dlo_map is defined elsewhere

//     float leaf_size = request->leaf_size;
//     std::string p = request->save_path;

//     std::cout << std::setprecision(2) << "Saving map to " << p + "/dlo_map.pcd" << "... ";
//     std::cout.flush();

//     // Voxelize map
//     pcl::VoxelGrid<PointType> vg;
//     vg.setLeafSize(leaf_size, leaf_size, leaf_size);
//     vg.setInputCloud(m);
//     vg.filter(*m);

//     // Save map
//     int ret = pcl::io::savePCDFileBinary(p + "/dlo_map.pcd", *m);
//     response->success = ret == 0;

//     if (response->success) {
//         std::cout << "done" << std::endl;
//     } else {
//         std::cout << "failed" << std::endl;
//     }
// }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}