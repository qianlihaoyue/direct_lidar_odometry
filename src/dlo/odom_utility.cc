#include "dlo/odom.h"
#include <sys/times.h>
#include <sys/vtimes.h>

void OdomNode::debugFun() {
    // Publish stuff to ROS
    publishPose();
    publishTransform();
    // Debug statements and publish custom DLO message
    if (debug_use_) debug();
}

void OdomNode::getParams() {
    // Topic
    declare_and_get_parameter<std::string>("odomNode.lidarType", "livox", lidarType);
    declare_and_get_parameter<std::string>("odomNode.lidarTopic", "livox/lidar", lidarTopic);
    declare_and_get_parameter<std::string>("odomNode.imuTopic", "livox/imu", imuTopic);

    // Frames
    declare_and_get_parameter<std::string>("odomNode.odom_frame", "odom", odom_frame);
    declare_and_get_parameter<std::string>("odomNode.child_frame", "base_link", child_frame);

    // IMU && Gravity alignment
    declare_and_get_parameter<bool>("imu.use", false, imu_use_);
    declare_and_get_parameter<bool>("imu.gravityAlign", false, gravity_align_);
    declare_and_get_parameter<int>("imu.calibTime", 3, imu_calib_time_);
    declare_and_get_parameter<int>("imu.bufferSize", 2000, imu_buffer_size_);

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
    declare_and_get_parameter<bool>("debugLog", false, debug_use_);

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

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) {
        globalPath.poses.push_back(pose_ros);
        pubPath->publish(globalPath);
    }
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

void OdomNode::debugInit() {
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
}

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
    // std::cout << "Position    [xyz]  :: " << pose[0] << " " << pose[1] << " " << pose[2] << std::endl;
    // std::cout << "Orientation [wxyz] :: " << rotq.w() << " " << rotq.x() << " " << rotq.y() << " " << rotq.z() << std::endl;
    std::cout << "Distance Traveled  :: " << length_traversed << " meters" << std::endl;
    // std::cout << "Distance to Origin :: " << sqrt(pow(pose[0] - origin[0], 2) + pow(pose[1] - origin[1], 2) + pow(pose[2] - origin[2], 2)) << " meters"
    //           << std::endl;

    std::cout << std::endl << std::right << std::setprecision(2) << std::fixed;
    std::cout << "Computation Time :: " << std::setfill(' ') << std::setw(6) << comp_times.back() * 1000. << " ms    // Avg: " << std::setw(5)
              << avg_comp_time * 1000. << std::endl;
    // std::cout << "Cores Utilized   :: " << std::setfill(' ') << std::setw(6) << (cpu_percent / 100.) * numProcessors << " cores // Avg: " << std::setw(5)
    //           << (avg_cpu_usage / 100.) * numProcessors << std::endl;
    std::cout << "CPU Load         :: " << std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: " << std::setw(5) << avg_cpu_usage << std::endl;
    std::cout << "RAM Allocation   :: " << std::setfill(' ') << std::setw(6) << resident_set / 1000. << " MB    // VSZ: " << vm_usage / 1000. << " MB "
              << std::endl;
}
