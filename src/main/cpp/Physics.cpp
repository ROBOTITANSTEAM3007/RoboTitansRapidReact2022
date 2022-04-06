#include <iostream>

#include "Physics.h"
#include "math.h"

double Physics::getVelocity(double distance, double height){//Returns the shot velocity by taking in distance and goal height.
    // height += offset_height;
    height -= exit_height * 2;
    distance += (exit_distance - cam_distance + offset_distance) * 2;

    distance /= 100;
    height /= 100;

    double velocity = sqrt(-(gravity * pow(distance, 2) / (2 * (height - distance * tan(exit_angle)) * pow(cos(exit_angle), 2) )));


    return velocity;
};

double Physics::getShotRPM(double velocity){//Returns the needed rpm of the shooter needed to reach the desired ball velocity.
    double radians_per_second = (2 * velocity) / (wheel_diameter/2);

    double rotaions_per_minute = radians_per_second * rps_to_rpm;
    rotaions_per_minute += rpm_drop;


    return rotaions_per_minute;
};