#include "dlo/odom.h"

void OdomNode::imuCB(const sensor_msgs::msg::Imu::ConstSharedPtr imu) {
    double ang_vel[3], lin_accel[3];

    // Get IMU samples
    ang_vel[0] = imu->angular_velocity.x;
    ang_vel[1] = imu->angular_velocity.y;
    ang_vel[2] = imu->angular_velocity.z;

    lin_accel[0] = imu->linear_acceleration.x;
    lin_accel[1] = imu->linear_acceleration.y;
    lin_accel[2] = imu->linear_acceleration.z;

    if (first_imu_time == 0.) first_imu_time = imu->header.stamp.sec;

    // IMU calibration procedure - do for three seconds
    if (!imu_calibrated) {
        static int num_samples = 0;
        static bool print = true;

        if ((imu->header.stamp.sec - first_imu_time) < imu_calib_time_) {
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
        imu_meas.stamp = imu->header.stamp.sec;

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

void OdomNode::gravityAlign() {
    std::cout << "start gravity align" << std::endl;
    // get average acceleration vector for 1 second and normalize
    Eigen::Vector3f lin_accel = Eigen::Vector3f::Zero();
    const double then = this->get_clock()->now().seconds();
    int n = 0;
    while ((this->get_clock()->now().seconds() - then) < 1.) {
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
