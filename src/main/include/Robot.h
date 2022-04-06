/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Debug.h"

#include <fstream>
#include <iostream>

#include <frc/TimedRobot.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/DigitalInput.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/Joystick.h>

#include <rev/CANPIDController.h>
#include <rev/CANSparkMax.h>

#include <cameraserver/CameraServer.h>

class Robot : public frc::TimedRobot {
 public:

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void DisabledPeriodic() override;

  void onShotRequest(double);
  void onDriveRequest();
  
  //Pin Variables
  
  //Joystick Pins
  unsigned const short driveJoystickID = 1;
  unsigned const short climbJoystickID = 0;

  //Spark Pins
  unsigned const short rightDriveMotorID = 1; //Default 1
  unsigned const short leftDrivemotorID = 2; //Default 2

  unsigned const short shootMotorID = 3; //Default 3
  unsigned const short indexMotorID = 1; //Default 1
  unsigned const short intakeMotorID = 6; //Default 6

  unsigned const short compressorID = 0; //Default 0

  //Button Variables
  unsigned const short shootHighButtonID = 1; //Default 6

  unsigned const short cameraButtonID = 7; //Default 7
  unsigned const short indexButtonID = 4; //Default 4
  unsigned const short intakeButtonID = 3; //Default 3

  unsigned const short increaseButtonID = 5; //Default 5
  unsigned const short decreaseButtonID = 4; //Default 5

  unsigned const short climbButtonID = 8; //Default 8


  //Constants

  //Joystick Constants
  unsigned const short joyStickDeadzone = 0.1;

  //Motor Constants
  unsigned const short intakeMotorSpeed = 1;
  unsigned const short indexMotorSpeed = 1;

  //Define Objects

  //Spark Objects
  rev::CANSparkMax m_shootMotor{shootMotorID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_leftDriveMotor{leftDrivemotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightDriveMotor{rightDriveMotorID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_intakeMotor{intakeMotorID, rev::CANSparkMax::MotorType::kBrushless};

  frc::Spark m_indexMotor{indexMotorID};

  //Drive Object
  frc::DifferentialDrive m_robotDrive{m_leftDriveMotor, m_rightDriveMotor};

  //PID Controller Objects
  rev::SparkMaxPIDController m_pidController = m_shootMotor.GetPIDController();
  rev::SparkMaxRelativeEncoder m_encoder = m_shootMotor.GetEncoder();

  //Joystick Objects
  frc::Joystick m_climbStick{climbJoystickID};
  frc::Joystick m_driveStick{driveJoystickID};

  //Pneumatics Objects
  frc::Compressor m_compressor{compressorID, frc::PneumaticsModuleType::CTREPCM};

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
