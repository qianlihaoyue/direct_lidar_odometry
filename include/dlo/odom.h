/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/dlo.h"

#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_box.h>
#include <boost/circular_buffer.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"

// #include <fast_gicp/gicp/fast_gicp.hpp>
// #include <fast_gicp/gicp/fast_gicp_st.hpp>
// #include <fast_gicp/gicp/fast_vgicp.hpp>

class OdomNode : public rclcpp::Node {
public:
    OdomNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    template <typename T>
    void declare_and_get_parameter(const std::string& param_yaml, const T& default_value, T& param_var) {
        this->declare_parameter<T>(param_yaml, default_value);
        this->get_parameter(param_yaml, param_var);
    }

    void icpLivoxCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr pc);
    void icpStdCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc);
    void icpCB(CloudPtr& current_scan);
    void imuCB(const sensor_msgs::msg::Imu::ConstSharedPtr imu);

    // ros::ServiceServer save_traj_srv;
    // bool saveTrajectory(direct_lidar_odometry::save_traj::Request& req,
    //                     direct_lidar_odometry::save_traj::Response& res);

    void getParams();

    void debugFun();
    void debugInit();
    void debug();

    void publishPose();
    void publishTransform();
    void publishKeyframe();

    void preprocessPoints();
    void initializeInputTarget();

    void initializeDLO();
    void gravityAlign();
    void integrateIMU();

    void setAdaptiveParams();
    void computeSpaciousness();

    void updateKeyframes();
    void computeConvexHull();
    void computeConcaveHull();
    void pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames);
    void getSubmapKeyframes();

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr icp_livox_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr icp_std_sub;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, kf_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    nav_msgs::msg::Path globalPath;

    Eigen::Vector3f origin;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> trajectory;
    std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, CloudPtr>> keyframes;
    std::vector<std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>> keyframe_normals;

    std::atomic<bool> dlo_initialized, imu_calibrated;

    std::string lidarType, lidarTopic, imuTopic;
    std::string odom_frame, child_frame;

    // CloudPtr original_scan{new CloudType()};
    CloudPtr current_scan{new CloudType()}, current_scan_t{new CloudType()};
    // CloudPtr keyframes_cloud{new CloudType()};
    CloudPtr keyframe_cloud{new CloudType()};
    int num_keyframes = 0;

    pcl::ConvexHull<PointType> convex_hull;
    pcl::ConcaveHull<PointType> concave_hull;
    std::vector<int> keyframe_convex, keyframe_concave;

    CloudPtr submap_cloud{new CloudType()};
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> submap_normals;

    std::vector<int> submap_kf_idx_curr, submap_kf_idx_prev;
    std::atomic<bool> submap_hasChanged;

    CloudPtr source_cloud, target_cloud;

    rclcpp::Time scan_stamp;

    double curr_frame_stamp, prev_frame_stamp;
    std::vector<double> comp_times;

    nano_gicp::NanoGICP<PointType, PointType> gicp_s2s;
    nano_gicp::NanoGICP<PointType, PointType> gicp;

    // fast_gicp::FastGICP<PointType, PointType> gicp_s2s;
    // fast_gicp::FastGICP<PointType, PointType> gicp;

    pcl::CropBox<PointType> crop;
    pcl::VoxelGrid<PointType> vf_scan, vf_submap;

    nav_msgs::msg::Odometry odom, kf;

    geometry_msgs::msg::PoseStamped pose_ros;

    Eigen::Matrix4f T, T_s2s, T_s2s_prev;
    Eigen::Quaternionf q_final;

    Eigen::Vector3f pose_s2s;
    Eigen::Matrix3f rotSO3_s2s;
    // Eigen::Quaternionf rotq_s2s;

    Eigen::Vector3f pose;
    Eigen::Matrix3f rotSO3;
    Eigen::Quaternionf rotq;

    ///// IMU
    Eigen::Matrix4f imu_SE3;

    struct XYZd {
        double x, y, z;
    };

    struct ImuBias {
        XYZd gyro, accel;
    };

    ImuBias imu_bias;

    struct ImuMeas {
        double stamp;
        XYZd ang_vel, lin_accel;
    };

    ImuMeas imu_meas;

    boost::circular_buffer<ImuMeas> imu_buffer;

    static bool comparatorImu(ImuMeas m1, ImuMeas m2) { return (m1.stamp < m2.stamp); };

    bool imu_use_;
    int imu_calib_time_;
    int imu_buffer_size_;
    double first_imu_time = 0;

    std::mutex mtx_imu;

    ////////

    struct Metrics {
        std::vector<float> spaciousness;
    };

    Metrics metrics;

    std::thread debug_thread;

    // debug
    std::string cpu_type;
    std::vector<double> cpu_percents;
    clock_t lastCPU, lastSysCPU, lastUserCPU;
    int numProcessors;

    // Parameters
    bool gravity_align_;

    double keyframe_thresh_dist_, keyframe_thresh_rot_;

    int submap_knn_, submap_kcv_, submap_kcc_;
    double submap_concave_alpha_;

    bool initial_pose_use_;
    Eigen::Vector3f initial_position_;
    Eigen::Quaternionf initial_orientation_;

    bool crop_use_;
    double crop_size_;

    bool vf_scan_use_;
    double vf_scan_res_;

    bool vf_submap_use_;
    double vf_submap_res_;

    bool adaptive_params_use_;
    bool debug_use_;

    int gicp_min_num_points_;

    int gicps2s_k_correspondences_;
    double gicps2s_max_corr_dist_;
    int gicps2s_max_iter_;
    double gicps2s_transformation_ep_;
    double gicps2s_euclidean_fitness_ep_;
    int gicps2s_ransac_iter_;
    double gicps2s_ransac_inlier_thresh_;

    int gicps2m_k_correspondences_;
    double gicps2m_max_corr_dist_;
    int gicps2m_max_iter_;
    double gicps2m_transformation_ep_;
    double gicps2m_euclidean_fitness_ep_;
    int gicps2m_ransac_iter_;
    double gicps2m_ransac_inlier_thresh_;
};
