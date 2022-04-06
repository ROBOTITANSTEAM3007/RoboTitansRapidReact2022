/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include "Debug.h"
#include "Climb.h"
#include "TankDrive.h"
#include "Limelight.h"
#include "Shooter.h"
#include "Game.h"

#include <cscore_oo.h>

//Climb Object
climb climbObject;

//Default PID coefficientss
double shotRPM = 2800;
double proportionalPIDConstant = 0.0006;
double intergralPIDConstant = 0.000001;
double derivativePIDConstant = 0;
double intergralZonePIDConstant = 0;
double feedForwardPIDConstant = 0;
double kMaxOutput = 1.0;
double kMinOutput = -1.0;

//Boolean Variables
bool indexActive = false;
bool intakeActive = false;
bool shootActive = false;
bool climbActive = false;
bool manualShooter = false;

double autoSteps = 0;

void Robot::onDriveRequest(double sensitivity = Robot::driveStickSensitivity){
  //Drive Robot
  if (abs(m_driveStick.GetTwist()) >= joyStickDeadzone || abs(m_driveStick.GetY()) >= joyStickDeadzone) {
    m_robotDrive.ArcadeDrive(m_driveStick.GetTwist() * sensitivity, -m_driveStick.GetY() * sensitivity);
  }
}

void Robot::onShotRequest(double goalHeight){
  Limelight::toggleCamera(0);

  Shooter::shoot(m_pidController, shotRPM);
  frc::SmartDashboard::PutNumber("Shot RPM", shotRPM);
  
  if (Limelight::visibleTarget()){
    double goalDistance = Limelight::getDistance();

    double steeringAdjust = 0;
    steeringAdjust = Shooter::alignTarget();
    
    double horizontalOffset = 0;
    horizontalOffset = Limelight::getInfo("tx");
    
    frc::SmartDashboard::PutNumber("Steering Adjust", 0);
    std::cout << steeringAdjust << std::endl;

    if (shotRPM > 0 && abs(horizontalOffset) < alignThreshold) {
      std::cout << shotRPM << std::endl;

      if (abs(goalDistance - shotDistance) > distanceThreshold) {
        if (goalDistance > shotDistance) {
          m_robotDrive.ArcadeDrive(0, minDrivePower); 
        } else if (goalDistance < shotDistance) {
          m_robotDrive.ArcadeDrive(0, -minDrivePower);
        }
      } else {
        double currentRPM = m_encoder.GetVelocity();
        bool speedReached = currentRPM > (shotRPM - rpmThreshold) && currentRPM < (shotRPM + rpmThreshold);
        if (speedReached) {
          indexActive = true;
        }
      }

    } else {
      m_robotDrive.ArcadeDrive(steeringAdjust, 0);
    }
  }
}

void Robot::RobotInit() {
  frc::CameraServer::StartAutomaticCapture();
  
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
  //Assign the PID variables
  m_pidController.SetP(proportionalPIDConstant);
  m_pidController.SetI(intergralPIDConstant);
  m_pidController.SetD(derivativePIDConstant);
  m_pidController.SetIZone(intergralZonePIDConstant);
  m_pidController.SetFF(feedForwardPIDConstant);
  m_pidController.SetOutputRange(kMinOutput, kMaxOutput);
  
  //Update the variables to the smartdashboard
  frc::SmartDashboard::PutBoolean("Far Enough", false);
  frc::SmartDashboard::PutNumber("Shot RPM", shotRPM);
  frc::SmartDashboard::PutNumber("Shooter Velocity", 0);
  frc::SmartDashboard::PutNumber("P Gain", proportionalPIDConstant);
  frc::SmartDashboard::PutNumber("I Gain", intergralPIDConstant);
  frc::SmartDashboard::PutNumber("D Gain", derivativePIDConstant);
  frc::SmartDashboard::PutNumber("I Zone", intergralZonePIDConstant);
  frc::SmartDashboard::PutNumber("Feed Forward", feedForwardPIDConstant);
  frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
  frc::SmartDashboard::PutNumber("Min Output", kMinOutput);

  frc::SmartDashboard::PutNumber("Distance", Limelight::getDistance());

  frc::SmartDashboard::PutBoolean("Target Visible", false);
  frc::SmartDashboard::PutBoolean("Toggled Camera", false);

  frc::SmartDashboard::PutBoolean("intakeActive", intakeActive);
  frc::SmartDashboard::PutBoolean("indexActive", indexActive); 
  frc::SmartDashboard::PutNumber("Steering Adjust", 0);

  //Custom debug class records debug log in /home/lvuser/DEBUG.txt
  //initialize and end are only run once and debug.out can be run as many times as you want.
  //debug.initialize("/home/lvuser/DEBUG.txt");
  //debug.out("test");
  //debug.end();

}
  
void Robot::RobotPeriodic() {
  cs::CvSink cvSink = frc::CameraServer::GetVideo();
  cs::CvSource outputStream = frc::CameraServer::PutVideo("Bottom_Of_Bot", 480, 480);
}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  m_autoSelected = m_chooser.GetSelected();
  m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  Limelight::toggleCamera(0);
  if (autoSteps < 20) {
    m_robotDrive.ArcadeDrive(0.2, 0);
  } else if (autoSteps < 30) {

    m_robotDrive.ArcadeDrive(-0.2, 0);
    m_intakeMotor.Set(intakeMotorSpeed);
  } else if (autoSteps > 0) {
    if (!Limelight::visibleTarget()) {
      m_robotDrive.ArcadeDrive(0, 0.05);
    } else {
      Robot::onShotRequest(top_goal);
    }
  }

  autoSteps += 1;

}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  //Read PID coefficients from SmartDashboard
  double s = frc::SmartDashboard::GetNumber("Shot RPM", 0);
  double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  double mx = frc::SmartDashboard::GetNumber("Max Output", 0);
  double mn = frc::SmartDashboard::GetNumber("Min Output", 0);


  //Update the Values
  frc::SmartDashboard::PutNumber("Distance", Limelight::getDistance());
  frc::SmartDashboard::PutBoolean("intakeActive", intakeActive);
  frc::SmartDashboard::PutBoolean("indexActive", indexActive);

  frc::SmartDashboard::PutNumber("Shooter Velocity", m_encoder.GetVelocity());

  //If PID coefficients on SmartDashboard have changed, write new values to controller
  if((s != shotRPM)) { shotRPM = s;}
  if((p != proportionalPIDConstant)) { m_pidController.SetP(p); proportionalPIDConstant = p;}
  if((i != intergralPIDConstant)) { m_pidController.SetI(i); intergralPIDConstant = i;}
  if((d != derivativePIDConstant)) { m_pidController.SetD(d); derivativePIDConstant = d;}
  if((iz != intergralZonePIDConstant)) { m_pidController.SetIZone(iz); intergralZonePIDConstant = iz;}
  if((ff != feedForwardPIDConstant)) { m_pidController.SetFF(ff); feedForwardPIDConstant = ff;}
  if((mx != kMaxOutput) || (mn != kMinOutput)) { 
    m_pidController.SetOutputRange(mn, mx);
    kMinOutput = mn;
    kMaxOutput = mx;
  }


  //Climbing
  if (m_climbStick.GetRawButtonPressed(climbButtonID)) {
  climbObject.prepareClimb(&climbActive);
  }
  
  if (climbActive) {
    onDriveRequest(climbStickSensitivity);
    climbObject.doClimb(&m_climbStick);
  } else {
    //Shooting
    if (!manualShooter) {
      if (m_driveStick.GetRawButton(shootHighButtonID)){
        std::cout << "Shoot High" << std::endl;
        Robot::onShotRequest(top_goal);
      } else {
        onDriveRequest();

        //While the shooter is not active
        indexActive = false;
        shootActive = false;
        Shooter::shoot(m_pidController, 0);
        // Limelight::toggleCamera(1);
      }
    } else {
      onDriveRequest();

      //Manual Shooter Controls
      if (m_driveStick.GetRawButtonPressed(increaseButtonID)){
        shotRPM += manualIncrement;
      } else if (m_driveStick.GetRawButtonPressed(decreaseButtonID)){
        shotRPM -= manualIncrement;
      }

      Shooter::shoot(m_pidController, shotRPM);
    }
    
    //Limelight
    if (m_driveStick.GetRawButtonPressed(cameraButtonID)){
      Limelight::toggleCamera();
    }

    //Index
    if (m_driveStick.GetRawButtonPressed(indexButtonID)) {
      indexActive = !indexActive;
    }

    if (indexActive) {
      m_indexMotor.Set(indexMotorSpeed);
    } else {
      m_indexMotor.Set(0.0);
    }

    //Intake
    if (m_driveStick.GetRawButtonPressed(intakeButtonID)) {
      intakeActive = !intakeActive;
    }

    if (intakeActive) {
      m_intakeMotor.Set(intakeMotorSpeed);
    } else {
      m_intakeMotor.Set(0.0);
    }
  }
}

void Robot::TestPeriodic() {
  if(m_climbStick.GetRawButtonPressed(climbButtonID)) {
    climbObject.prepareClimb(&climbActive);
  }

  onDriveRequest();

  if (climbActive) {
    climbObject.doClimb(&m_climbStick);
  }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif