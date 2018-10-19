#ifndef CONFIG_H
#define CONFIG_H

#include "RobotState.h"

extern RobotState currentState;

#define HEARTBEAT_LED 13
#define HEARTBEAT_DISABLED_HZ 1
#define HEARTBEAT_AUTO_HZ 10
#define HEARTBEAT_TELEOP_HZ 5

// R9 transmitter
// Timeout before we consider the transmitter disconnected
#define TX_TIMEOUT 2000
#define TX_SERIALPORT Serial1

// Loop rates
#define MAIN_LOOP_HZ 100
#define XBEE_LOOP_HZ 3
#define ENCODER_UPDATE_HZ 50
#define DEBUG_LOOP_HZ 5

#define ESC_PWM 3 // Throttle
#define SERVO_PWM 4


// Steering trim
#define STEERING_OFFSET -0.2

#define THROTTLE_MIN 0.1
#define STEERING_MIN 0.05

#define LEFT_ENCODER 20
#define RIGHT_ENCODER 22

// Distance from front wheel to back wheel
#define TRACKLENGTH 0.3175
// Distance from left wheel to right wheel
#define TRACKWIDTH 0.254

const constexpr double kIMUAccelVariance[2] = {100, 100};
const constexpr double kIMUOrientationVariance[2] = {1e-5, 1e-5};
const constexpr double kDistancePerTick = (0.075 * 3.14) / 10.0;

#endif
