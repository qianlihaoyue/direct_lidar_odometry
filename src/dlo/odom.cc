#include "dlo/odom.h"

OdomNode::OdomNode(const rclcpp::NodeOptions& options) : Node("dlo_odom_node", options) {
    getParams();

    dlo_initialized = imu_calibrated = false;

    if (lidarType == "livox")
        icp_livox_sub = create_subscription<livox_ros_driver2::msg::CustomMsg>(lidarTopic, rclcpp::SensorDataQoS(),
                                                                               std::bind(&OdomNode::icpLivoxCB, this, std::placeholders::_1));
    else
        icp_std_sub = create_subscription<sensor_msgs::msg::PointCloud2>(lidarTopic, rclcpp::SensorDataQoS(),
                                                                         std::bind(&OdomNode::icpStdCB, this, std::placeholders::_1));

    if (imu_use_)
        imu_sub = create_subscription<sensor_msgs::msg::Imu>(imuTopic, rclcpp::SensorDataQoS(), std::bind(&OdomNode::imuCB, this, std::placeholders::_1));

    odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    kf_pub = create_publisher<nav_msgs::msg::Odometry>("kfs", 1);
    pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);
    keyframe_pub = create_publisher<sensor_msgs::msg::PointCloud2>("keyframe", 1);

    pubPath = create_publisher<nav_msgs::msg::Path>("path", 1);
    globalPath.header.stamp = scan_stamp;
    globalPath.header.frame_id = odom_frame;
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
    // rotq_s2s = Eigen::Quaternionf(1., 0., 0., 0.);

    pose = Eigen::Vector3f(0., 0., 0.);
    rotq = Eigen::Quaternionf(1., 0., 0., 0.);

    imu_SE3 = Eigen::Matrix4f::Identity();

    imu_bias.gyro.x = imu_bias.gyro.y = imu_bias.gyro.z = 0.;
    imu_bias.accel.x = imu_bias.accel.y = imu_bias.accel.z = 0.;

    imu_meas.stamp = 0.;
    imu_meas.ang_vel.x = imu_meas.ang_vel.y = imu_meas.ang_vel.z = 0.;
    imu_meas.lin_accel.x = imu_meas.lin_accel.y = imu_meas.lin_accel.z = 0.;

    imu_buffer.set_capacity(imu_buffer_size_);

    submap_hasChanged = true;

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

    debugInit();
    RCLCPP_INFO(rclcpp::get_logger("DirectLidarOdometry"), "DLO Odom Node Initialized");
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
    // *original_scan = *current_scan;

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

static void moveFromCustomMsg(livox_ros_driver2::msg::CustomMsg& Msg, CloudType& cloud) {
    cloud.clear();
    cloud.reserve(Msg.point_num);
    PointType point;

    cloud.header.frame_id = Msg.header.frame_id;
    cloud.header.stamp = (uint64_t)((Msg.header.stamp.sec * 1e9 + Msg.header.stamp.nanosec) / 1000);
    // cloud.header.seq=Msg.header.seq;

    for (uint i = 0; i < Msg.point_num - 1; i++) {
        point.x = Msg.points[i].x;
        point.y = Msg.points[i].y;
        point.z = Msg.points[i].z;
        point.intensity = Msg.points[i].reflectivity;
        // point.tag = Msg.points[i].tag;
        // point.time = Msg.points[i].offset_time * 1e-9;
        // point.ring = Msg.points[i].line;
        cloud.push_back(point);
    }
}

void OdomNode::icpLivoxCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr pc) {
    scan_stamp = pc->header.stamp;
    curr_frame_stamp = pc->header.stamp.sec;
    current_scan = CloudPtr(new CloudType);
    moveFromCustomMsg(*pc, *current_scan);
    icpCB(current_scan);
}

void OdomNode::icpStdCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc) {
    scan_stamp = pc->header.stamp;
    curr_frame_stamp = pc->header.stamp.sec;
    current_scan = CloudPtr(new CloudType);
    pcl::fromROSMsg(*pc, *current_scan);
    icpCB(current_scan);
}

void OdomNode::icpCB(CloudPtr& current_scan) {
    // If there are too few points in the pointcloud, try again
    if (current_scan->points.size() < gicp_min_num_points_) {
        RCLCPP_WARN(rclcpp::get_logger("DirectLidarOdometry"), "Low number of points!");
        return;
    }

    // DLO Initialization procedures (IMU calib, gravity align)
    if (!dlo_initialized) {
        initializeDLO();
        return;
    }

    int t_pre, t_ff, t_fs, t_all;
    TIMER_CREATE(tim_main);
    TIMER_CREATE(tim_all);

    // Preprocess points
    tim_main.tic();
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
    // setInputSources();
    // set the input source for the S2S gicp
    // this builds the KdTree of the source cloud
    // this does not build the KdTree for s2m because force_no_update is true
    gicp_s2s.setInputSource(current_scan);
    // set pcl::Registration input source for S2M gicp using custom NanoGICP function
    gicp.registerInputSource(current_scan);
    // now set the KdTree of S2M gicp using previously built KdTree
    gicp.source_kdtree_ = gicp_s2s.source_kdtree_;
    gicp.source_covs_.clear();

    t_pre = tim_main.toc();

    // Get the next pose via IMU + S2S + S2M

    ////// FRAME-TO-FRAME PROCEDURE
    tim_main.tic();
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
    T_s2s = T_s2s_prev * T_S2S;
    T_s2s_prev = T_s2s;

    pose_s2s << T_s2s(0, 3), T_s2s(1, 3), T_s2s(2, 3);
    rotSO3_s2s << T_s2s(0, 0), T_s2s(0, 1), T_s2s(0, 2), T_s2s(1, 0), T_s2s(1, 1), T_s2s(1, 2), T_s2s(2, 0), T_s2s(2, 1), T_s2s(2, 2);
    // rotq_s2s = Eigen::Quaternionf(rotSO3_s2s).normalized();

    // reuse covariances from s2s for s2m
    gicp.source_covs_ = gicp_s2s.source_covs_;
    // Swap source and target (which also swaps KdTrees internally) for next S2S
    gicp_s2s.swapSourceAndTarget();

    t_ff = tim_main.toc();

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

    t_fs = tim_main.toc();

    // Update current keyframe poses and map
    updateKeyframes();

    // Update trajectory
    trajectory.push_back(std::make_pair(pose, rotq));

    // Update next time stamp
    prev_frame_stamp = curr_frame_stamp;

    t_all = tim_all.toc();

    static int tim_cnt = 0;
    if (tim_cnt > 5 || t_all > 30) {
        tim_cnt = 0;
        std::cout << "t_pre: " << t_pre << " t_ff: " << t_ff << " t_fs: " << t_fs << " t_all: " << t_all << std::endl;
    }
    ++tim_cnt;

    // Update some statistics
    comp_times.push_back(t_all / 1000.0);

    this->debug_thread = std::thread(&OdomNode::debugFun, this);
    this->debug_thread.detach();
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
        CloudPtr cur_kf(new CloudType);
        *cur_kf = *current_scan_t;
        keyframes.push_back(std::make_pair(std::make_pair(pose, rotq), cur_kf));

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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNode>());
    rclcpp::shutdown();
    return 0;
}
