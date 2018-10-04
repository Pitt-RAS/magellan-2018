#include "config.h"
#include <ros.h>
#include "Rate.h"
#include "Robot.h"

ros::NodeHandle nh;
Rate loop_rate(MAIN_LOOP_HZ);

void setup() {
    pinMode(20, INPUT_PULLUP);
    pinMode(13, OUTPUT);
    while (1)
        digitalWrite(13, digitalRead(20));


    nh.initNode();

    // Use full teensy ADC resolution
    analogReadResolution(13);

    Robot robot(nh);
    while (true) {
        robot.Update();
        loop_rate.Sleep();
    }
}

void loop() {}
