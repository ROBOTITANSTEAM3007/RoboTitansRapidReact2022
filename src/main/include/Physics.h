#pragma once

#include "Game.h"

#include "Shooter.h"

#include "frc/smartdashboard/SmartDashboard.h"

class Physics{
public:
    static double getVelocity(double, double);
    static double getMinDistance(double);
    static double getHeight(double);
    static double getTime(double, double, double);
    static double getShotRPM(double);
    static bool farEnough(double, double);

};