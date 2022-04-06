#pragma once

#include "Game.h"

#include "Shooter.h"

#include "frc/smartdashboard/SmartDashboard.h"

class Physics{
public:
    static double getVelocity(double, double);
    static double getShotRPM(double);
};