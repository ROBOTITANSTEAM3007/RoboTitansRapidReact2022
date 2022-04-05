#pragma once


#include <iostream>
#include "Game.h"
#include "Physics.h"
#include "Limelight.h"


#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>

#include <rev/CANPIDController.h>
#include <rev/CANSparkMax.h>

#define motorMaxRPM 4000 //Motor max RPM (The fastest the motor can spin)
#define rpmThreshold 10
#define driveProportion 0.125
#define minAdjustment -0.3
#define maxAdjustment 0.3
#define shotThreshold 1
#define minDrivePower 0.5

#define shotDistance 240 // in cm
#define distanceThreshold 10 // in cm
#define alignThreshold 1 //in degrees

#define alignSpeed 0.3
#define RPMScalar 1.4

#define rpm_drop 500

class Shooter{
public:
    static void shoot(rev::SparkMaxPIDController&, double);
    static double alignTarget();

};
