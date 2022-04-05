#include "Shooter.h"


double prevHorizontialOffset = 0;

double min(double a, double b){
    if (a < b){
        return a;
    }
    return b;
};

double max(double a, double b){
    if (a > b){
        return a;
    }
    return b;
};

//Shoots the balls at the desired rpm
void Shooter::shoot(rev::SparkMaxPIDController& pidController, double rpm){
    double shotRPM = min(motorMaxRPM, rpm);

    pidController.SetReference(shotRPM, rev::ControlType::kVelocity);
    // pidController.SetReference(shotRPM, rev::ControlType::kPosition)
};

//Aligns the robot with the goal.
double Shooter::alignTarget(){
    double horizontalOffset = 0;
    horizontalOffset += Limelight::getInfo("tx");//-29.8 to +29.8 degrees;
    
    double steeringAdjust = 0;
    steeringAdjust = horizontalOffset * driveProportion;

    steeringAdjust = min(max(steeringAdjust, minAdjustment), maxAdjustment);

    frc::SmartDashboard::PutNumber("Steering Adjust", steeringAdjust);

    return steeringAdjust;

};