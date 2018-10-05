/**
 * Teensy IMU for PittRAS Magellan
 * Adapted from PittRAS Sailbot 
 * Original Authors: Andrew Lobos and Kaylene Stocking
 * Modified by: Xinke Chen
 * 2018-10-03
 */
#ifndef MAGELLAN_IMU_H_
#define MAGELLAN_IMU_H_

#include <ros.h>
#include <Adafruit_BNO055.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <magellan_core/IMUState.h>

class MagellanIMU{
public:
    MagellanIMU(ros::NodeHandle &imu_handle);
    double getHeading();
    void update();
private:
    ros::NodeHandle node_handle;
    ros::Publisher imu_publisher;
    ros::Publisher imu_state_publisher;
    Adafruit_BNO055 imu;
    sensor_msgs::Imu imu_msg;
    magellan_core::IMUState imu_state_msg;
    adafruit_bno055_offsets_t sensor_offsets;
    // IMU constants
    const int16_t ACCEL_X = -7;
    const int16_t ACCEL_Y = -30;
    const int16_t ACCEL_Z = 24;
    const int16_t MAG_X = 35;
    const int16_t MAG_Y = -112;
    const int16_t MAG_Z = -536;
    const int16_t GYRO_X = -2;
    const int16_t GYRO_Y = 0;
    const int16_t GYRO_Z = -1;
    const int16_t ACCEL_RADIUS = 1000;
    const int16_t MAG_RADIUS = 896;
};

#endif