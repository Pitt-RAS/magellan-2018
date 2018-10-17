#include "Drivetrain.h"
#include "config.h"

Drivetrain::Drivetrain(ros::NodeHandle& nh) :
        last_commanded_percent_(0),
        nh_(nh),
        throttle_pwm_(ESC_PWM),
        steering_pwm_(SERVO_PWM),
        steering_angle_msg_(),
        steering_angle_publisher_("/platform/turning_radius", &steering_angle_msg_) {
    throttle_pwm_.ConfigLowLimit(THROTTLE_MIN);
    steering_pwm_.ConfigOffset(STEERING_OFFSET);
    steering_pwm_.ConfigLowLimit(STEERING_MIN);

    nh.advertise(steering_angle_publisher_);
}

void Drivetrain::SetThrottlePercent(double percent) {
    throttle_pwm_.Set(percent);

    if ( fabs(percent) > 0.1 )
        last_commanded_percent_ = percent;
}

void Drivetrain::SetSteeringPercent(double percent) {
    steering_pwm_.Set(percent);
    steering_angle_msg_.data = GetTurningRadius(percent);
    steering_angle_publisher_.publish(&steering_angle_msg_);
}

void Drivetrain::SetSteeringAngle(double angle) {
    SetSteeringPercent(GetPercentForSteeringAngle(angle));
}

double Drivetrain::GetSteeringAngleForPercent(double percent) {
    return percent * M_PI / 6.0;
}

double Drivetrain::GetPercentForSteeringAngle(double angle) {
    return angle / 30.0;
}

double Drivetrain::GetTurningRadius(double percent) {
    if ( percent == 0 )
        return 0;

    double angle = GetSteeringAngleForPercent(percent);
    return TRACKLENGTH * (1.0 / tan(angle)) + (TRACKWIDTH / 2.0);
}

bool Drivetrain::DirectionIsForward() {
    return last_commanded_percent_ >= 0;
}
