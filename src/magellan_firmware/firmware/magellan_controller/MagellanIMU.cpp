#include "MagellanIMU.h"

MagellanIMU::MagellanIMU(ros::NodeHandle &imu_handle) : 
    node_handle(imu_handle),
    imu_publisher("imu", &imu_msg),
    imu_state_publisher("imu_state", &imu_state_msg),
    imu(55) {
    imu.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
    // Setting IMU calibration data
    sensor_offsets.accel_offset_x = ACCEL_X;
    sensor_offsets.accel_offset_y = ACCEL_Y;
    sensor_offsets.accel_offset_z = ACCEL_Z;
    sensor_offsets.mag_offset_x = MAG_X;
    sensor_offsets.mag_offset_y = MAG_Y;
    sensor_offsets.mag_offset_z = MAG_Z;
    sensor_offsets.gyro_offset_x = GYRO_X;
    sensor_offsets.gyro_offset_y = GYRO_Y;
    sensor_offsets.gyro_offset_z = GYRO_Z;
    sensor_offsets.accel_radius = ACCEL_RADIUS;
    sensor_offsets.mag_radius = MAG_RADIUS;
    imu.setSensorOffsets(sensor_offsets);
    // ROS configuration
    imu_msg.header.frame_id = "Magellan_IMU";
    node_handle.advertise(imu_publisher);
    node_handle.advertise(imu_state_publisher);
}

// Returns IMU heading
double MagellanIMU::getHeading() {
    return imu.getVector(Adafruit_BNO055::VECTOR_EULER).z(); // what does vector.z() correspond with ? 
}

// Publishes quaternion data and physical status of IMU
void MagellanIMU::update() {
    uint8_t system = 0, gyro = 0, accel = 0, mag = 0;
    imu.getCalibration(&system, &gyro, &accel, &mag);
    // Set IMU State messages
    imu_state_msg.system = system;
    imu_state_msg.gyro = gyro;
    imu_state_msg.accel = accel;
    imu_state_msg.mag = accel;
    imu_state_publisher.publish(&imu_state_msg);
    // Publishes IMU data if there is a change
    if(system > 0 && gyro > 0 && mag > 0) {
        // Retrieve data from IMU
        imu::Quaternion quaternion = imu.getQuat();
        imu::Vector<3> linear_accel = imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu::Vector<3> angular_velocity = imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        // Setting message data
        imu_msg.header.stamp = node_handle.now();
        imu_msg.orientation.x = quaternion.x();
        imu_msg.orientation.y = quaternion.y();
        imu_msg.orientation.z = quaternion.z();
        imu_msg.orientation.w = quaternion.w();
        imu_msg.linear_acceleration.x = linear_accel.x();
        imu_msg.linear_acceleration.y = linear_accel.y();
        imu_msg.linear_acceleration.z = linear_accel.z();
        imu_msg.angular_velocity.x = angular_velocity.x();
        imu_msg.angular_velocity.y = angular_velocity.y();
        imu_msg.angular_velocity.z = angular_velocity.z();
        imu_publisher.publish(&imu_msg);
    }
}
