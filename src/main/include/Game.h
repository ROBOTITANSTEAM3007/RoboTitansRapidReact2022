#pragma once

#define offset_height 0.00//The vertical offset of the goal(cm)
#define offset_distance 0.00//The horizontal offset of the goal(cm)


#define cam_height 58.42//Height of camera from the floor(cm)
#define exit_height 81.28//Height of the shot origin from the floor(cm)

#define cam_distance 49.53//Distance from the back of the bot to the camera(cm)
#define exit_distance 22.86//Distance from the back of the bot to the shot exit(cm)


#define top_goal 264.00/*Height of the UPPER goal from the floor(cm)*/
#define low_goal 104.00/*Height of the LOWER goal from the floor(cm)*/


#define cam_angle 26.00/*Angle of camera(degrees)(originally 27)*/ * deg_to_rad
#define exit_angle 60.00 /*Angle of the shooter(degrees)*/ * deg_to_rad

#define ball_diameter 0.2400//Diameter of the ball(m)
#define wheel_diameter 0.1524//Diameter of the shooting wheel(m)

#define wheel_distance 0.6604//Distance between the left and right wheels(m)

#define rps_to_rpm 1 / (2*PI) * 60
#define deg_to_rad PI/180

#define gravity 9.807//Gravity(m/s/s)

#define PI 3.141592653589793//PI
