/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>
#include <frc/drive/DifferentialDrive.h>
#include <iostream>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Compressor.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include <fstream>
#include "Debug.h"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <rev/CANPIDController.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/StartEndCommand.h>



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
  
  //Pin Variables
  
  //Joystick Pins
  unsigned const short driveJoystickID = 0;
  unsigned const short controlJoystickID = 1;

  //Spark Pins
  unsigned const short rightDriveMotorID = 1; //Default 1
  unsigned const short leftDrivemotorID = 2; //Default 2

  unsigned const short shootMotorID = 3; //Default 3
  unsigned const short indexMotorID = 1; //Default 1
  unsigned const short intakeMotorID = 6; //Default 6

  //Button Variables
  

  unsigned const short shootHighButtonID = 1; //Default 6
  unsigned const short shootLowButtonID = 5; //Default 5

  unsigned const short cameraButtonID = 7; //Default 7
  unsigned const short indexButtonID = 4; //Default 4
  unsigned const short intakeButtonID = 3; //Default 8

  //Constants

  //Joystick Constants
  unsigned const short joyStickDeadzone = 0.1;
// <<<<<<< Test
// =======
//   unsigned const short controlStickSensitivty = 0.75;
// >>>>>>> Test

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
// <<<<<<< Test
// =======
//   rev::SparkMaxPIDController m_leftPidController = m_leftDriveMotor.GetPIDController();
//   rev::SparkMaxPIDController m_rightPidController = m_rightDriveMotor.GetPIDController();
//   rev::SparkMaxRelativeEncoder m_leftEncoder = m_leftDriveMotor.GetEncoder();
//   rev::SparkMaxRelativeEncoder m_rightEncoder = m_rightDriveMotor.GetEncoder();
//   // frc::PIDController m_drivePidController = m_robotDrive;
// >>>>>>> Test
  rev::SparkMaxPIDController m_pidController = m_shootMotor.GetPIDController();
  rev::SparkMaxRelativeEncoder m_encoder = m_shootMotor.GetEncoder();

  //Joystick Objects
  frc::Joystick m_controlStick{controlJoystickID};
  frc::Joystick m_driveStick{driveJoystickID};

  //Pneumatics Objects
  frc::Compressor m_compressor{0, frc::PneumaticsModuleType::CTREPCM};


 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
