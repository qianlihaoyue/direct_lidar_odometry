#include "dlo/odom.h"
#include <sys/times.h>
#include <sys/vtimes.h>

OdomNode::OdomNode(const rclcpp::NodeOptions& options) : Node("dlo_odom_node", options) {
    getParams();

    dlo_initialized = imu_calibrated = false;

    icp_sub = create_subscription<sensor_msgs::msg::PointCloud2>(lidarTopic, rclcpp::SensorDataQoS(), std::bind(&OdomNode::icpCB, this, std::placeholders::_1));
    imu_sub = create_subscription<sensor_msgs::msg::Imu>(imuTopic, rclcpp::SensorDataQoS(), std::bind(&OdomNode::imuCB, this, std::placeholders::_1));

    odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    kf_pub = create_publisher<nav_msgs::msg::Odometry>("kfs", 1);  // 1,true);
    pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);
    keyframe_pub = create_publisher<sensor_msgs::msg::PointCloud2>("keyframe", 1);  // 1,true);
    // save_traj_srv = create_service("SaveTraj", &OdomNode::saveTrajectory, this);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto& opose = odom.pose.pose;
    opose.position.x = opose.position.y = opose.position.z = 0.;
    opose.orientation.x = opose.orientation.y = opose.orientation.z = 0.;
    opose.orientation.w = 1.;
    odom.pose.covariance = {0.};

    origin = Eigen::Vector3f(0., 0., 0.);

    T = T_s2s = T_s2s_prev = Eigen::Matrix4f::Identity();

    pose_s2s = Eigen::Vector3f(0., 0., 0.);
    rotq_s2s = Eigen::Quaternionf(1., 0., 0., 0.);

    pose = Eigen::Vector3f(0., 0., 0.);
    rotq = Eigen::Quaternionf(1., 0., 0., 0.);

    imu_SE3 = Eigen::Matrix4f::Identity();

    imu_bias.gyro.x = imu_bias.gyro.y = imu_bias.gyro.z = 0.;
    imu_bias.accel.x = imu_bias.accel.y = imu_bias.accel.z = 0.;

    imu_meas.stamp = 0.;
    imu_meas.ang_vel.x = imu_meas.ang_vel.y = imu_meas.ang_vel.z = 0.;
    imu_meas.lin_accel.x = imu_meas.lin_accel.y = imu_meas.lin_accel.z = 0.;

    imu_buffer.set_capacity(imu_buffer_size_);
    first_imu_time = 0.;

    original_scan = CloudPtr(new CloudType);
    current_scan = CloudPtr(new CloudType);
    current_scan_t = CloudPtr(new CloudType);

    keyframe_cloud = CloudPtr(new CloudType);
    // keyframes_cloud = CloudPtr(new CloudType);
    num_keyframes = 0;

    submap_cloud = CloudPtr(new CloudType);
    submap_hasChanged = true;
    submap_kf_idx_prev.clear();

    source_cloud = target_cloud = nullptr;

    convex_hull.setDimension(3);
    concave_hull.setDimension(3);
    concave_hull.setAlpha(keyframe_thresh_dist_);
    concave_hull.setKeepInformation(true);

    gicp_s2s.setCorrespondenceRandomness(gicps2s_k_correspondences_);
    gicp_s2s.setMaxCorrespondenceDistance(gicps2s_max_corr_dist_);
    gicp_s2s.setMaximumIterations(gicps2s_max_iter_);
    gicp_s2s.setTransformationEpsilon(gicps2s_transformation_ep_);
    gicp_s2s.setEuclideanFitnessEpsilon(gicps2s_euclidean_fitness_ep_);
    gicp_s2s.setRANSACIterations(gicps2s_ransac_iter_);
    gicp_s2s.setRANSACOutlierRejectionThreshold(gicps2s_ransac_inlier_thresh_);

    gicp.setCorrespondenceRandomness(gicps2m_k_correspondences_);
    gicp.setMaxCorrespondenceDistance(gicps2m_max_corr_dist_);
    gicp.setMaximumIterations(gicps2m_max_iter_);
    gicp.setTransformationEpsilon(gicps2m_transformation_ep_);
    gicp.setEuclideanFitnessEpsilon(gicps2m_euclidean_fitness_ep_);
    gicp.setRANSACIterations(gicps2m_ransac_iter_);
    gicp.setRANSACOutlierRejectionThreshold(gicps2m_ransac_inlier_thresh_);

    pcl::Registration<PointType, PointType>::KdTreeReciprocalPtr temp;
    gicp_s2s.setSearchMethodSource(temp, true);
    gicp_s2s.setSearchMethodTarget(temp, true);
    gicp.setSearchMethodSource(temp, true);
    gicp.setSearchMethodTarget(temp, true);

    crop.setNegative(true);
    crop.setMin(Eigen::Vector4f(-crop_size_, -crop_size_, -crop_size_, 1.0));
    crop.setMax(Eigen::Vector4f(crop_size_, crop_size_, crop_size_, 1.0));

    vf_scan.setLeafSize(vf_scan_res_, vf_scan_res_, vf_scan_res_);
    vf_submap.setLeafSize(vf_submap_res_, vf_submap_res_, vf_submap_res_);

    metrics.spaciousness.push_back(0.);

    // debug
    // char CPUBrandString[0x40];
    // memset(CPUBrandString, 0, sizeof(CPUBrandString));
    // cpu_type = "";
    FILE* file;
    struct tms timeSample;
    char line[128];
    lastCPU = times(&timeSample);
    lastSysCPU = timeSample.tms_stime;
    lastUserCPU = timeSample.tms_utime;
    file = fopen("/proc/cpuinfo", "r");
    numProcessors = 0;
    while (fgets(line, 128, file) != NULL)
        if (strncmp(line, "processor", 9) == 0) numProcessors++;
    fclose(file);

    RCLCPP_INFO(rclcpp::get_logger("DirectLidarOdometry"), "DLO Odom Node Initialized");
}

void OdomNode::getParams() {
    // Topic
    declare_and_get_parameter<std::string>("odomNode.lidarTopic", "livox/lidar", lidarTopic);
    declare_and_get_parameter<std::string>("odomNode.imuTopic", "livox/imu", imuTopic);

    // Frames
    declare_and_get_parameter<std::string>("odomNode.odom_frame", "odom", odom_frame);
    declare_and_get_parameter<std::string>("odomNode.child_frame", "base_link", child_frame);

    // Gravity alignment
    declare_and_get_parameter<bool>("gravityAlign", false, gravity_align_);

    // Keyframe Threshold
    declare_and_get_parameter<double>("odomNode.keyframe.threshD", 0.1, keyframe_thresh_dist_);
    declare_and_get_parameter<double>("odomNode.keyframe.threshR", 1.0, keyframe_thresh_rot_);

    // Submap
    declare_and_get_parameter<int>("odomNode.submap.keyframe.knn", 10, submap_knn_);
    declare_and_get_parameter<int>("odomNode.submap.keyframe.kcv", 10, submap_kcv_);
    declare_and_get_parameter<int>("odomNode.submap.keyframe.kcc", 10, submap_kcc_);

    // Initial Position
    declare_and_get_parameter<bool>("odomNode.initialPose.use", false, initial_pose_use_);

    double px, py, pz, qx, qy, qz, qw;
    declare_and_get_parameter<double>("odomNode.initialPose.position.x", 0.0, px);
    declare_and_get_parameter<double>("odomNode.initialPose.position.y", 0.0, py);
    declare_and_get_parameter<double>("odomNode.initialPose.position.z", 0.0, pz);
    declare_and_get_parameter<double>("odomNode.initialPose.orientation.w", 1.0, qw);
    declare_and_get_parameter<double>("odomNode.initialPose.orientation.x", 0.0, qx);
    declare_and_get_parameter<double>("odomNode.initialPose.orientation.y", 0.0, qy);
    declare_and_get_parameter<double>("odomNode.initialPose.orientation.z", 0.0, qz);
    initial_position_ = Eigen::Vector3f(px, py, pz);
    initial_orientation_ = Eigen::Quaternionf(qw, qx, qy, qz);

    // Crop Box Filter
    declare_and_get_parameter<bool>("odomNode.preprocessing.cropBoxFilter.use", false, crop_use_);
    declare_and_get_parameter<double>("odomNode.preprocessing.cropBoxFilter.size", 1.0, crop_size_);

    // Voxel Grid Filter
    declare_and_get_parameter<bool>("odomNode.preprocessing.voxelFilter.scan.use", true, vf_scan_use_);
    declare_and_get_parameter<double>("odomNode.preprocessing.voxelFilter.scan.res", 0.05, vf_scan_res_);
    declare_and_get_parameter<bool>("odomNode.preprocessing.voxelFilter.submap.use", false, vf_submap_use_);
    declare_and_get_parameter<double>("odomNode.preprocessing.voxelFilter.submap.res", 0.1, vf_submap_res_);

    // Adaptive Parameters
    declare_and_get_parameter<bool>("adaptiveParams", false, adaptive_params_use_);

    // IMU
    declare_and_get_parameter<bool>("imu", false, imu_use_);
    declare_and_get_parameter<int>("odomNode.imu.calibTime", 3, imu_calib_time_);
    declare_and_get_parameter<int>("odomNode.imu.bufferSize", 2000, imu_buffer_size_);

    // GICP
    declare_and_get_parameter<int>("odomNode.gicp.minNumPoints", 100, gicp_min_num_points_);
    declare_and_get_parameter<int>("odomNode.gicp.s2s.kCorrespondences", 20, gicps2s_k_correspondences_);
    declare_and_get_parameter<double>("odomNode.gicp.s2s.maxCorrespondenceDistance", std::sqrt(std::numeric_limits<double>::max()), gicps2s_max_corr_dist_);
    declare_and_get_parameter<int>("odomNode.gicp.s2s.maxIterations", 64, gicps2s_max_iter_);
    declare_and_get_parameter<double>("odomNode.gicp.s2s.transformationEpsilon", 0.0005, gicps2s_transformation_ep_);
    declare_and_get_parameter<double>("odomNode.gicp.s2s.euclideanFitnessEpsilon", -std::numeric_limits<double>::max(), gicps2s_euclidean_fitness_ep_);
    declare_and_get_parameter<int>("odomNode.gicp.s2s.ransac.iterations", 0, gicps2s_ransac_iter_);
    declare_and_get_parameter<double>("odomNode.gicp.s2s.ransac.outlierRejectionThresh", 0.05, gicps2s_ransac_inlier_thresh_);
    declare_and_get_parameter<int>("odomNode.gicp.s2m.kCorrespondences", 20, gicps2m_k_correspondences_);
    declare_and_get_parameter<double>("odomNode.gicp.s2m.maxCorrespondenceDistance", std::sqrt(std::numeric_limits<double>::max()), gicps2m_max_corr_dist_);
    declare_and_get_parameter<int>("odomNode.gicp.s2m.maxIterations", 64, gicps2m_max_iter_);
    declare_and_get_parameter<double>("odomNode.gicp.s2m.transformationEpsilon", 0.0005, gicps2m_transformation_ep_);
    declare_and_get_parameter<double>("odomNode.gicp.s2m.euclideanFitnessEpsilon", -std::numeric_limits<double>::max(), gicps2m_euclidean_fitness_ep_);
    declare_and_get_parameter<int>("odomNode.gicp.s2m.ransac.iterations", 0, gicps2m_ransac_iter_);
    declare_and_get_parameter<double>("odomNode.gicp.s2m.ransac.outlierRejectionThresh", 0.05, gicps2m_ransac_inlier_thresh_);
}

void OdomNode::publishPose() {
    // Sign flip check
    static Eigen::Quaternionf q_diff{1., 0., 0., 0.};
    static Eigen::Quaternionf q_last{1., 0., 0., 0.};

    q_diff = q_last.conjugate() * rotq;

    // If q_diff has negative real part then there was a sign flip
    if (q_diff.w() < 0) {
        rotq.w() = -rotq.w();
        rotq.vec() = -rotq.vec();
    }

    q_last = rotq;

    odom.pose.pose.position.x = pose[0], odom.pose.pose.position.y = pose[1], odom.pose.pose.position.z = pose[2];
    odom.pose.pose.orientation.w = rotq.w();
    odom.pose.pose.orientation.x = rotq.x(), odom.pose.pose.orientation.y = rotq.y(), odom.pose.pose.orientation.z = rotq.z();

    odom.header.stamp = scan_stamp;
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = child_frame;
    odom_pub->publish(odom);

    pose_ros.header.stamp = scan_stamp;
    pose_ros.header.frame_id = odom_frame;

    // pose_ros.pose = odom.pose.pose;
    pose_ros.pose.position.x = pose[0], pose_ros.pose.position.y = pose[1], pose_ros.pose.position.z = pose[2];
    pose_ros.pose.orientation.w = rotq.w();
    pose_ros.pose.orientation.x = rotq.x(), pose_ros.pose.orientation.y = rotq.y(), pose_ros.pose.orientation.z = rotq.z();

    pose_pub->publish(pose_ros);
}

void OdomNode::publishTransform() {
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = scan_stamp;
    transformStamped.header.frame_id = odom_frame;
    transformStamped.child_frame_id = child_frame;

    transformStamped.transform.translation.x = pose[0];
    transformStamped.transform.translation.y = pose[1];
    transformStamped.transform.translation.z = pose[2];

    transformStamped.transform.rotation.w = rotq.w();
    transformStamped.transform.rotation.x = rotq.x();
    transformStamped.transform.rotation.y = rotq.y();
    transformStamped.transform.rotation.z = rotq.z();

    tf_broadcaster_->sendTransform(transformStamped);
}

void OdomNode::publishKeyframe() {
    // Publish keyframe pose
    kf.header.stamp = scan_stamp;
    kf.header.frame_id = odom_frame;
    kf.child_frame_id = child_frame;

    kf.pose.pose.position.x = pose[0], kf.pose.pose.position.y = pose[1], kf.pose.pose.position.z = pose[2];
    kf.pose.pose.orientation.w = rotq.w();
    kf.pose.pose.orientation.x = rotq.x(), kf.pose.pose.orientation.y = rotq.y(), kf.pose.pose.orientation.z = rotq.z();
    kf_pub->publish(kf);

    // Publish keyframe scan
    if (keyframe_cloud->points.size()) {
        sensor_msgs::msg::PointCloud2 keyframe_cloud_ros;
        pcl::toROSMsg(*keyframe_cloud, keyframe_cloud_ros);
        keyframe_cloud_ros.header.stamp = scan_stamp;
        keyframe_cloud_ros.header.frame_id = odom_frame;
        keyframe_pub->publish(keyframe_cloud_ros);
    }
}

void OdomNode::preprocessPoints() {
    *original_scan = *current_scan;

    // Remove NaNs
    std::vector<int> idx;
    current_scan->is_dense = false;
    pcl::removeNaNFromPointCloud(*current_scan, *current_scan, idx);

    // Crop Box Filter
    if (crop_use_) {
        crop.setInputCloud(current_scan);
        crop.filter(*current_scan);
    }
    // Voxel Grid Filter
    if (vf_scan_use_) {
        vf_scan.setInputCloud(current_scan);
        vf_scan.filter(*current_scan);
    }
}

void OdomNode::initializeInputTarget() {
    prev_frame_stamp = curr_frame_stamp;

    // Convert ros message
    target_cloud = CloudPtr(new CloudType);
    target_cloud = current_scan;
    gicp_s2s.setInputTarget(target_cloud);
    gicp_s2s.calculateTargetCovariances();

    // initialize keyframes
    CloudPtr first_keyframe(new CloudType);
    pcl::transformPointCloud(*target_cloud, *first_keyframe, T);

    // voxelization for submap
    if (vf_submap_use_) {
        vf_submap.setInputCloud(first_keyframe);
        vf_submap.filter(*first_keyframe);
    }

    // keep history of keyframes
    keyframes.push_back(std::make_pair(std::make_pair(pose, rotq), first_keyframe));
    // *keyframes_cloud += *first_keyframe;
    *keyframe_cloud = *first_keyframe;

    // compute kdtree and keyframe normals (use gicp_s2s input source as temporary storage because it will be overwritten by setInputSources())
    gicp_s2s.setInputSource(keyframe_cloud);
    gicp_s2s.calculateSourceCovariances();
    keyframe_normals.push_back(gicp_s2s.getSourceCovariances());

    publishKeyframe();
    ++num_keyframes;
}

void OdomNode::setInputSources() {
    // set the input source for the S2S gicp
    // this builds the KdTree of the source cloud
    // this does not build the KdTree for s2m because force_no_update is true
    gicp_s2s.setInputSource(current_scan);
    // set pcl::Registration input source for S2M gicp using custom NanoGICP function
    gicp.registerInputSource(current_scan);
    // now set the KdTree of S2M gicp using previously built KdTree
    gicp.source_kdtree_ = gicp_s2s.source_kdtree_;
    gicp.source_covs_.clear();
}

void OdomNode::gravityAlign() {
    // get average acceleration vector for 1 second and normalize
    Eigen::Vector3f lin_accel = Eigen::Vector3f::Zero();
    const double then = rclcpp::Clock().now().seconds();
    int n = 0;
    while ((rclcpp::Clock().now().seconds() - then) < 1.) {
        lin_accel[0] += imu_meas.lin_accel.x;
        lin_accel[1] += imu_meas.lin_accel.y;
        lin_accel[2] += imu_meas.lin_accel.z;
        ++n;
    }
    lin_accel[0] /= n, lin_accel[1] /= n, lin_accel[2] /= n;

    // normalize
    double lin_norm = sqrt(pow(lin_accel[0], 2) + pow(lin_accel[1], 2) + pow(lin_accel[2], 2));
    lin_accel[0] /= lin_norm, lin_accel[1] /= lin_norm, lin_accel[2] /= lin_norm;

    // define gravity vector (assume point downwards)
    Eigen::Vector3f grav;
    grav << 0, 0, 1;

    // calculate angle between the two vectors
    Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(lin_accel, grav).normalized();

    // set gravity aligned orientation
    rotq = grav_q;
    T.block(0, 0, 3, 3) = rotq.toRotationMatrix();
    T_s2s.block(0, 0, 3, 3) = rotq.toRotationMatrix();
    T_s2s_prev.block(0, 0, 3, 3) = rotq.toRotationMatrix();

    // rpy
    auto euler = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
    double yaw = euler[0] * (180.0 / M_PI);
    double pitch = euler[1] * (180.0 / M_PI);
    double roll = euler[2] * (180.0 / M_PI);

    std::cout << "done" << std::endl;
    std::cout << "  Roll [deg]: " << roll << std::endl;
    std::cout << "  Pitch [deg]: " << pitch << std::endl << std::endl;
}

void OdomNode::initializeDLO() {
    // Calibrate IMU
    if (!imu_calibrated && imu_use_) return;

    // Gravity Align
    if (gravity_align_ && imu_use_ && imu_calibrated && !initial_pose_use_) {
        std::cout << "Aligning to gravity... ";
        std::cout.flush();
        gravityAlign();
    }

    // Use initial known pose
    if (initial_pose_use_) {
        std::cout << "Setting known initial pose... ";
        std::cout.flush();

        // set known position
        pose = initial_position_;
        T.block(0, 3, 3, 1) = pose;
        T_s2s.block(0, 3, 3, 1) = pose;
        T_s2s_prev.block(0, 3, 3, 1) = pose;
        origin = initial_position_;

        // set known orientation
        rotq = initial_orientation_;
        T.block(0, 0, 3, 3) = rotq.toRotationMatrix();
        T_s2s.block(0, 0, 3, 3) = rotq.toRotationMatrix();
        T_s2s_prev.block(0, 0, 3, 3) = rotq.toRotationMatrix();

        std::cout << "done" << std::endl << std::endl;
    }

    dlo_initialized = true;
    std::cout << "DLO initialized! Starting localization..." << std::endl;
}

void OdomNode::icpCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc) {
    double then = this->get_clock()->now().seconds();
    scan_stamp = pc->header.stamp;
    curr_frame_stamp = pc->header.stamp.sec;

    // If there are too few points in the pointcloud, try again
    current_scan = CloudPtr(new CloudType);
    pcl::fromROSMsg(*pc, *current_scan);
    if (current_scan->points.size() < gicp_min_num_points_) {
        RCLCPP_WARN(rclcpp::get_logger("DirectLidarOdometry"), "Low number of points!");
        return;
    }

    // DLO Initialization procedures (IMU calib, gravity align)
    if (!dlo_initialized) {
        initializeDLO();
        return;
    }

    // Preprocess points
    preprocessPoints();

    // Compute Metrics
    computeSpaciousness();

    // Set Adaptive Parameters
    if (adaptive_params_use_) setAdaptiveParams();

    // Set initial frame as target
    if (target_cloud == nullptr) {
        initializeInputTarget();
        return;
    }

    // Set source frame
    source_cloud = CloudPtr(new CloudType);
    source_cloud = current_scan;

    // Set new frame as input source for both gicp objects
    setInputSources();

    // Get the next pose via IMU + S2S + S2M
    getNextPose();

    // Update current keyframe poses and map
    updateKeyframes();

    // Update trajectory
    trajectory.push_back(std::make_pair(pose, rotq));

    // Update next time stamp
    prev_frame_stamp = curr_frame_stamp;

    // Update some statistics
    comp_times.push_back(this->get_clock()->now().seconds() - then);

    this->debug_thread = std::thread(&OdomNode::debugFun, this);
    this->debug_thread.detach();
}

void OdomNode::debugFun() {
    // Publish stuff to ROS
    publishPose();
    publishTransform();
    // Debug statements and publish custom DLO message
    debug();
}

void OdomNode::imuCB(const sensor_msgs::msg::Imu::ConstSharedPtr imu) {
    if (!imu_use_) return;

    double ang_vel[3], lin_accel[3];

    // Get IMU samples
    ang_vel[0] = imu->angular_velocity.x;
    ang_vel[1] = imu->angular_velocity.y;
    ang_vel[2] = imu->angular_velocity.z;

    lin_accel[0] = imu->linear_acceleration.x;
    lin_accel[1] = imu->linear_acceleration.y;
    lin_accel[2] = imu->linear_acceleration.z;

    if (first_imu_time == 0.) first_imu_time = imu->header.stamp.nanosec * 1e-9;

    // IMU calibration procedure - do for three seconds
    if (!imu_calibrated) {
        static int num_samples = 0;
        static bool print = true;

        if ((imu->header.stamp.nanosec * 1e-9 - first_imu_time) < imu_calib_time_) {
            num_samples++;

            imu_bias.gyro.x += ang_vel[0], imu_bias.gyro.y += ang_vel[1], imu_bias.gyro.z += ang_vel[2];
            imu_bias.accel.x += lin_accel[0], imu_bias.accel.y += lin_accel[1], imu_bias.accel.z += lin_accel[2];
            if (print) {
                std::cout << "Calibrating IMU for " << imu_calib_time_ << " seconds... ";
                std::cout.flush();
                print = false;
            }
        } else {
            imu_bias.gyro.x /= num_samples, imu_bias.gyro.y /= num_samples, imu_bias.gyro.z /= num_samples;
            imu_bias.accel.x /= num_samples, imu_bias.accel.y /= num_samples, imu_bias.accel.z /= num_samples;
            imu_calibrated = true;

            std::cout << "done" << std::endl;
            std::cout << "  Gyro biases [xyz]: " << imu_bias.gyro.x << ", " << imu_bias.gyro.y << ", " << imu_bias.gyro.z << std::endl << std::endl;
        }

    } else {
        // Apply the calibrated bias to the new IMU measurements
        imu_meas.stamp = imu->header.stamp.nanosec * 1e-9;

        imu_meas.ang_vel.x = ang_vel[0] - imu_bias.gyro.x;
        imu_meas.ang_vel.y = ang_vel[1] - imu_bias.gyro.y;
        imu_meas.ang_vel.z = ang_vel[2] - imu_bias.gyro.z;

        imu_meas.lin_accel.x = lin_accel[0];
        imu_meas.lin_accel.y = lin_accel[1];
        imu_meas.lin_accel.z = lin_accel[2];

        // Store into circular buffer
        mtx_imu.lock();
        imu_buffer.push_front(imu_meas);
        mtx_imu.unlock();
    }
}

void OdomNode::getNextPose() {
    ////// FRAME-TO-FRAME PROCEDURE

    // Align using IMU prior if available
    CloudPtr aligned(new CloudType);

    if (imu_use_) {
        integrateIMU();
        gicp_s2s.align(*aligned, imu_SE3);
    } else {
        gicp_s2s.align(*aligned);
    }

    // Get the local S2S transform
    Eigen::Matrix4f T_S2S = gicp_s2s.getFinalTransformation();
    // Get the global S2S transform
    // propagateS2S(T_S2S);
    T_s2s = T_s2s_prev * T_S2S;
    T_s2s_prev = T_s2s;

    pose_s2s << T_s2s(0, 3), T_s2s(1, 3), T_s2s(2, 3);
    rotSO3_s2s << T_s2s(0, 0), T_s2s(0, 1), T_s2s(0, 2), T_s2s(1, 0), T_s2s(1, 1), T_s2s(1, 2), T_s2s(2, 0), T_s2s(2, 1), T_s2s(2, 2);
    rotq_s2s = Eigen::Quaternionf(rotSO3_s2s).normalized();

    // reuse covariances from s2s for s2m
    gicp.source_covs_ = gicp_s2s.source_covs_;
    // Swap source and target (which also swaps KdTrees internally) for next S2S
    gicp_s2s.swapSourceAndTarget();

    ////// FRAME-TO-SUBMAP

    // Get current global submap
    getSubmapKeyframes();
    if (submap_hasChanged) {
        // Set the current global submap as the target cloud
        gicp.setInputTarget(submap_cloud);
        // Set target cloud's normals as submap normals
        gicp.setTargetCovariances(submap_normals);
    }
    // Align with current submap with global S2S transformation as initial guess
    gicp.align(*aligned, T_s2s);
    // Get final transformation in global frame
    T = gicp.getFinalTransformation();
    // Update the S2S transform for next propagation
    T_s2s_prev = T;
    // Update next global pose
    // Both source and target clouds are in the global frame now, so tranformation is global
    // Propagate S2M Alignment
    pose << T(0, 3), T(1, 3), T(2, 3);
    rotSO3 << T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2);
    rotq = Eigen::Quaternionf(rotSO3).normalized();

    // Set next target cloud as current source cloud
    *target_cloud = *source_cloud;
}

void OdomNode::integrateIMU() {
    // Extract IMU data between the two frames
    std::vector<ImuMeas> imu_frame;

    for (const auto& i : imu_buffer) {
        // IMU data between two frames is when:
        //   current frame's timestamp minus imu timestamp is positive
        //   previous frame's timestamp minus imu timestamp is negative
        double curr_frame_imu_dt = curr_frame_stamp - i.stamp;
        double prev_frame_imu_dt = prev_frame_stamp - i.stamp;

        if (curr_frame_imu_dt >= 0. && prev_frame_imu_dt <= 0.) imu_frame.push_back(i);
    }

    // Sort measurements by time
    std::sort(imu_frame.begin(), imu_frame.end(), comparatorImu);

    // Relative IMU integration of gyro and accelerometer
    double curr_imu_stamp = 0., prev_imu_stamp = 0., dt;

    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    for (uint32_t i = 0; i < imu_frame.size(); ++i) {
        if (prev_imu_stamp == 0.) {
            prev_imu_stamp = imu_frame[i].stamp;
            continue;
        }

        // Calculate difference in imu measurement times IN SECONDS
        curr_imu_stamp = imu_frame[i].stamp;
        dt = curr_imu_stamp - prev_imu_stamp;
        prev_imu_stamp = curr_imu_stamp;

        // Relative gyro propagation quaternion dynamics
        Eigen::Quaternionf qq = q;
        q.w() -= 0.5 * (qq.x() * imu_frame[i].ang_vel.x + qq.y() * imu_frame[i].ang_vel.y + qq.z() * imu_frame[i].ang_vel.z) * dt;
        q.x() += 0.5 * (qq.w() * imu_frame[i].ang_vel.x - qq.z() * imu_frame[i].ang_vel.y + qq.y() * imu_frame[i].ang_vel.z) * dt;
        q.y() += 0.5 * (qq.z() * imu_frame[i].ang_vel.x + qq.w() * imu_frame[i].ang_vel.y - qq.x() * imu_frame[i].ang_vel.z) * dt;
        q.z() += 0.5 * (qq.x() * imu_frame[i].ang_vel.y - qq.y() * imu_frame[i].ang_vel.x + qq.w() * imu_frame[i].ang_vel.z) * dt;
    }

    // Store IMU guess
    imu_SE3 = Eigen::Matrix4f::Identity();
    imu_SE3.block(0, 0, 3, 3) = q.normalized().toRotationMatrix();
}

// Compute Spaciousness of Current Scan
void OdomNode::computeSpaciousness() {
    // compute range of points
    std::vector<float> ds;

    for (int i = 0; i <= current_scan->points.size(); i++) {
        float d = std::sqrt(pow(current_scan->points[i].x, 2) + pow(current_scan->points[i].y, 2) + pow(current_scan->points[i].z, 2));
        ds.push_back(d);
    }

    // median
    std::nth_element(ds.begin(), ds.begin() + ds.size() / 2, ds.end());
    float median_curr = ds[ds.size() / 2];
    static float median_prev = median_curr;
    float median_lpf = 0.95 * median_prev + 0.05 * median_curr;
    median_prev = median_lpf;

    metrics.spaciousness.push_back(median_lpf);
}

//  Convex Hull of Keyframes
void OdomNode::computeConvexHull() {
    // at least 4 keyframes for convex hull
    if (num_keyframes < 4) return;

    // create a pointcloud with points at keyframes
    CloudPtr cloud = CloudPtr(new CloudType);

    for (const auto& k : keyframes) {
        PointType pt;
        pt.x = k.first.first[0], pt.y = k.first.first[1], pt.z = k.first.first[2];
        cloud->push_back(pt);
    }

    // calculate the convex hull of the point cloud
    convex_hull.setInputCloud(cloud);
    // get the indices of the keyframes on the convex hull
    CloudPtr convex_points = CloudPtr(new CloudType);
    convex_hull.reconstruct(*convex_points);
    pcl::PointIndices::Ptr convex_hull_point_idx = pcl::PointIndices::Ptr(new pcl::PointIndices);
    convex_hull.getHullPointIndices(*convex_hull_point_idx);

    keyframe_convex.clear();
    for (int i = 0; i < convex_hull_point_idx->indices.size(); ++i) {
        keyframe_convex.push_back(convex_hull_point_idx->indices[i]);
    }
}

// Concave Hull of Keyframes
void OdomNode::computeConcaveHull() {
    // at least 5 keyframes for concave hull
    if (num_keyframes < 5) return;
    // create a pointcloud with points at keyframes
    CloudPtr cloud = CloudPtr(new CloudType);

    for (const auto& k : keyframes) {
        PointType pt;
        pt.x = k.first.first[0], pt.y = k.first.first[1], pt.z = k.first.first[2];
        cloud->push_back(pt);
    }

    // calculate the concave hull of the point cloud
    concave_hull.setInputCloud(cloud);

    // get the indices of the keyframes on the concave hull
    CloudPtr concave_points = CloudPtr(new CloudType);
    concave_hull.reconstruct(*concave_points);

    pcl::PointIndices::Ptr concave_hull_point_idx = pcl::PointIndices::Ptr(new pcl::PointIndices);
    concave_hull.getHullPointIndices(*concave_hull_point_idx);

    keyframe_concave.clear();
    for (int i = 0; i < concave_hull_point_idx->indices.size(); ++i) {
        keyframe_concave.push_back(concave_hull_point_idx->indices[i]);
    }
}

void OdomNode::updateKeyframes() {
    // transform point cloud
    current_scan_t = CloudPtr(new CloudType);
    pcl::transformPointCloud(*current_scan, *current_scan_t, T);

    // calculate difference in pose and rotation to all poses in trajectory
    float closest_d = std::numeric_limits<float>::infinity();
    int closest_idx = 0, keyframes_idx = 0, num_nearby = 0;

    for (const auto& k : keyframes) {
        // calculate distance between current pose and pose in keyframes
        float delta_d = sqrt(pow(pose[0] - k.first.first[0], 2) + pow(pose[1] - k.first.first[1], 2) + pow(pose[2] - k.first.first[2], 2));
        // count the number nearby current pose
        if (delta_d <= keyframe_thresh_dist_ * 1.5) ++num_nearby;
        // store into variable
        if (delta_d < closest_d) {
            closest_d = delta_d;
            closest_idx = keyframes_idx;
        }
        keyframes_idx++;
    }

    // get closest pose and corresponding rotation
    Eigen::Vector3f closest_pose = keyframes[closest_idx].first.first;
    Eigen::Quaternionf closest_pose_r = keyframes[closest_idx].first.second;

    // calculate distance between current pose and closest pose from above
    float dd = sqrt(pow(pose[0] - closest_pose[0], 2) + pow(pose[1] - closest_pose[1], 2) + pow(pose[2] - closest_pose[2], 2));

    // calculate difference in orientation
    Eigen::Quaternionf dq = rotq * (closest_pose_r.inverse());

    float theta_rad = 2. * atan2(sqrt(pow(dq.x(), 2) + pow(dq.y(), 2) + pow(dq.z(), 2)), dq.w());
    float theta_deg = theta_rad * (180.0 / M_PI);

    // update keyframe
    bool newKeyframe = false;
    if (abs(dd) > keyframe_thresh_dist_ || abs(theta_deg) > keyframe_thresh_rot_) newKeyframe = true;
    if (abs(dd) <= keyframe_thresh_dist_) newKeyframe = false;
    if (abs(dd) <= keyframe_thresh_dist_ && abs(theta_deg) > keyframe_thresh_rot_ && num_nearby <= 1) newKeyframe = true;

    if (newKeyframe) {
        ++num_keyframes;

        // voxelization for submap
        if (vf_submap_use_) {
            vf_submap.setInputCloud(current_scan_t);
            vf_submap.filter(*current_scan_t);
        }

        // update keyframe vector
        keyframes.push_back(std::make_pair(std::make_pair(pose, rotq), current_scan_t));

        // compute kdtree and keyframe normals (use gicp_s2s input source as temporary storage because it will be overwritten by setInputSources())
        // *keyframes_cloud += *current_scan_t;
        *keyframe_cloud = *current_scan_t;

        gicp_s2s.setInputSource(keyframe_cloud);
        gicp_s2s.calculateSourceCovariances();
        keyframe_normals.push_back(gicp_s2s.getSourceCovariances());

        publishKeyframe();
    }
}

void OdomNode::setAdaptiveParams() {
    // Set Keyframe Thresh from Spaciousness Metric
    if (metrics.spaciousness.back() > 20.0) {
        keyframe_thresh_dist_ = 10.0;
    } else if (metrics.spaciousness.back() > 10.0 && metrics.spaciousness.back() <= 20.0) {
        keyframe_thresh_dist_ = 5.0;
    } else if (metrics.spaciousness.back() > 5.0 && metrics.spaciousness.back() <= 10.0) {
        keyframe_thresh_dist_ = 1.0;
    } else if (metrics.spaciousness.back() <= 5.0) {
        keyframe_thresh_dist_ = 0.5;
    }

    // set concave hull alpha
    concave_hull.setAlpha(keyframe_thresh_dist_);
}

void OdomNode::pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames) {
    // make sure dists is not empty
    if (!dists.size()) return;

    // maintain max heap of at most k elements
    std::priority_queue<float> pq;

    for (auto d : dists) {
        if (pq.size() >= k && pq.top() > d) {
            pq.push(d);
            pq.pop();
        } else if (pq.size() < k) {
            pq.push(d);
        }
    }

    // get the kth smallest element, which should be at the top of the heap
    float kth_element = pq.top();

    // get all elements smaller or equal to the kth smallest element
    for (int i = 0; i < dists.size(); ++i) {
        if (dists[i] <= kth_element) submap_kf_idx_curr.push_back(frames[i]);
    }
}

//  Get Submap using Nearest Neighbor Keyframes
void OdomNode::getSubmapKeyframes() {
    // clear vector of keyframe indices to use for submap
    submap_kf_idx_curr.clear();

    ////// TOP K NEAREST NEIGHBORS FROM ALL KEYFRAMES

    // calculate distance between current pose and poses in keyframe set
    std::vector<float> ds;
    std::vector<int> keyframe_nn;
    int i = 0;
    Eigen::Vector3f curr_pose = T_s2s.block(0, 3, 3, 1);

    for (const auto& k : keyframes) {
        float d = sqrt(pow(curr_pose[0] - k.first.first[0], 2) + pow(curr_pose[1] - k.first.first[1], 2) + pow(curr_pose[2] - k.first.first[2], 2));
        ds.push_back(d);
        keyframe_nn.push_back(i);
        i++;
    }

    // get indices for top K nearest neighbor keyframe poses
    pushSubmapIndices(ds, submap_knn_, keyframe_nn);

    ////// TOP K NEAREST NEIGHBORS FROM CONVEX HULL

    // get convex hull indices
    computeConvexHull();

    // get distances for each keyframe on convex hull
    std::vector<float> convex_ds;
    for (const auto& c : keyframe_convex) convex_ds.push_back(ds[c]);

    // get indicies for top kNN for convex hull
    pushSubmapIndices(convex_ds, submap_kcv_, keyframe_convex);

    ////// TOP K NEAREST NEIGHBORS FROM CONCAVE HULL

    // get concave hull indices
    computeConcaveHull();

    // get distances for each keyframe on concave hull
    std::vector<float> concave_ds;
    for (const auto& c : keyframe_concave) concave_ds.push_back(ds[c]);

    // get indicies for top kNN for convex hull
    pushSubmapIndices(concave_ds, submap_kcc_, keyframe_concave);

    ////// BUILD SUBMAP

    // concatenate all submap clouds and normals
    std::sort(submap_kf_idx_curr.begin(), submap_kf_idx_curr.end());
    auto last = std::unique(submap_kf_idx_curr.begin(), submap_kf_idx_curr.end());
    submap_kf_idx_curr.erase(last, submap_kf_idx_curr.end());

    // sort current and previous submap kf list of indices
    std::sort(submap_kf_idx_curr.begin(), submap_kf_idx_curr.end());
    std::sort(submap_kf_idx_prev.begin(), submap_kf_idx_prev.end());

    // check if submap has changed from previous iteration
    if (submap_kf_idx_curr == submap_kf_idx_prev) {
        submap_hasChanged = false;
    } else {
        submap_hasChanged = true;

        // reinitialize submap cloud, normals
        CloudPtr submap_cloud_(boost::make_shared<CloudType>());
        submap_normals.clear();

        for (auto k : submap_kf_idx_curr) {
            // create current submap cloud
            *submap_cloud_ += *keyframes[k].second;
            // grab corresponding submap cloud's normals
            submap_normals.insert(std::end(submap_normals), std::begin(keyframe_normals[k]), std::end(keyframe_normals[k]));
        }

        submap_cloud = submap_cloud_;
        submap_kf_idx_prev = submap_kf_idx_curr;
    }
}

// bool OdomNode::saveTrajectory(direct_lidar_odometry::save_traj::Request& req,
//                                    direct_lidar_odometry::save_traj::Response& res) {
//   std::string kittipath = req.save_path + "/kitti_traj.txt";
//   std::ofstream out_kitti(kittipath);

//   std::cout << std::setprecision(2) << "Saving KITTI trajectory to " << kittipath << "... "; std::cout.flush();

//   for (const auto& pose : trajectory) {
//     const auto& t = pose.first;
//     const auto& q = pose.second;
//     // Write to Kitti Format
//     auto R = q.normalized().toRotationMatrix();
//     out_kitti << std::fixed << std::setprecision(9)
//       << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << t.x() << " "
//       << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " " << t.y() << " "
//       << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " " << t.z() << "\n";
//   }

//   std::cout << "done" << std::endl;
//   res.success = true;
//   return res.success;
// }

void OdomNode::debug() {
    // Total length traversed
    double length_traversed = 0.;
    Eigen::Vector3f p_curr = Eigen::Vector3f(0., 0., 0.);
    Eigen::Vector3f p_prev = Eigen::Vector3f(0., 0., 0.);
    for (const auto& t : trajectory) {
        if (p_prev == Eigen::Vector3f(0., 0., 0.)) {
            p_prev = t.first;
            continue;
        }
        p_curr = t.first;
        double l = sqrt(pow(p_curr[0] - p_prev[0], 2) + pow(p_curr[1] - p_prev[1], 2) + pow(p_curr[2] - p_prev[2], 2));

        if (l >= 0.05) {
            length_traversed += l;
            p_prev = p_curr;
        }
    }

    if (length_traversed == 0) publishKeyframe();

    // Average computation time
    double avg_comp_time = std::accumulate(comp_times.begin(), comp_times.end(), 0.0) / comp_times.size();

    // RAM Usage
    double vm_usage = 0.0, resident_set = 0.0;
    std::ifstream stat_stream("/proc/self/stat", std::ios_base::in);  // get info from proc directory
    std::string pid, comm, state, ppid, pgrp, session, tty_nr;
    std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
    std::string utime, stime, cutime, cstime, priority, nice;
    std::string num_threads, itrealvalue, starttime;
    unsigned long vsize;
    long rss;
    stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt >> utime >> stime >>
        cutime >> cstime >> priority >> nice >> num_threads >> itrealvalue >> starttime >> vsize >> rss;  // don't care about the rest
    stat_stream.close();
    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;  // for x86-64 is configured to use 2MB pages
    vm_usage = vsize / 1024.0;
    resident_set = rss * page_size_kb;

    // CPU Usage
    struct tms timeSample;
    clock_t now;
    double cpu_percent;
    now = times(&timeSample);
    if (now <= lastCPU || timeSample.tms_stime < lastSysCPU || timeSample.tms_utime < lastUserCPU) {
        cpu_percent = -1.0;
    } else {
        cpu_percent = (timeSample.tms_stime - lastSysCPU) + (timeSample.tms_utime - lastUserCPU);
        cpu_percent /= (now - lastCPU);
        cpu_percent /= numProcessors;
        cpu_percent *= 100.;
    }
    lastCPU = now;
    lastSysCPU = timeSample.tms_stime;
    lastUserCPU = timeSample.tms_utime;
    cpu_percents.push_back(cpu_percent);
    double avg_cpu_usage = std::accumulate(cpu_percents.begin(), cpu_percents.end(), 0.0) / cpu_percents.size();

    // Print to terminal
    printf("\033[2J\033[1;1H");

    std::cout << std::endl << "==== Direct LiDAR Odometry" << " ====" << std::endl;

    // if (!cpu_type.empty()) std::cout << std::endl << cpu_type << " x " << numProcessors << std::endl;

    std::cout << std::endl << std::setprecision(4) << std::fixed;
    std::cout << "Position    [xyz]  :: " << pose[0] << " " << pose[1] << " " << pose[2] << std::endl;
    std::cout << "Orientation [wxyz] :: " << rotq.w() << " " << rotq.x() << " " << rotq.y() << " " << rotq.z() << std::endl;
    std::cout << "Distance Traveled  :: " << length_traversed << " meters" << std::endl;
    std::cout << "Distance to Origin :: " << sqrt(pow(pose[0] - origin[0], 2) + pow(pose[1] - origin[1], 2) + pow(pose[2] - origin[2], 2)) << " meters"
              << std::endl;

    std::cout << std::endl << std::right << std::setprecision(2) << std::fixed;
    std::cout << "Computation Time :: " << std::setfill(' ') << std::setw(6) << comp_times.back() * 1000. << " ms    // Avg: " << std::setw(5)
              << avg_comp_time * 1000. << std::endl;
    std::cout << "Cores Utilized   :: " << std::setfill(' ') << std::setw(6) << (cpu_percent / 100.) * numProcessors << " cores // Avg: " << std::setw(5)
              << (avg_cpu_usage / 100.) * numProcessors << std::endl;
    std::cout << "CPU Load         :: " << std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: " << std::setw(5) << avg_cpu_usage << std::endl;
    std::cout << "RAM Allocation   :: " << std::setfill(' ') << std::setw(6) << resident_set / 1000. << " MB    // VSZ: " << vm_usage / 1000. << " MB "
              << std::endl;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNode>());
    rclcpp::shutdown();
    return 0;
}
