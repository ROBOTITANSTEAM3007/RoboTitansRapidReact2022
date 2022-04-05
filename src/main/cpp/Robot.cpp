/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"


#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>
#include <frc/drive/DifferentialDrive.h>
#include <iostream>
#include <cameraserver/CameraServer.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Compressor.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include <fstream>
#include <cscore_oo.h>
#include "Debug.h"
#include "Climb.h"
#include "TankDrive.h"
#include "Limelight.h"
#include "Shooter.h"
#include "Game.h"
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/button/JoystickButton.h>

  //Put pin numbers as variables here.
  unsigned const short driveStickID = 0;
  unsigned const short controlStickID = 1;
  unsigned const short shootHighButtonID = 4;
  unsigned const short shootLowButtonID = 5;
  unsigned const short cameraButtonID = 6;
  //The Lead motor is the one in front of the lagging motor.
  static unsigned const short leadRightSparkID = 1;
  static unsigned const short leadLeftSparkID = 2;
  static unsigned const short shooterMotorID = 3;
  static unsigned const short intakeMotorID = 6;
  //Define Spark and Spark Max objects
  rev::CANSparkMax m_leftLeadingMotor{leadLeftSparkID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadingMotor{leadRightSparkID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_shooterMotor{shooterMotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_intakeMotor{intakeMotorID, rev::CANSparkMax::MotorType::kBrushless};
  frc::Spark m_indexBelt{1};
  //will follow the leading motors.
  frc::DifferentialDrive m_robotDrive{m_leftLeadingMotor, m_rightLeadingMotor};
  //Fine control differential drive object is needed for other joystick
  tankDrive m_robotControl(&m_leftLeadingMotor, &m_rightLeadingMotor);
  //Instantiate the left joystick
  frc::Joystick m_climbStick{driveStickID};
  //Instantiate the control joystick
  frc::Joystick m_controlStick{controlStickID};
  //Instantiate shooter speed variable for manipulation in Robot Periodic
  float shooterSpeed = 1.0;
  //create a Digital Input object to get grip detection
  frc::DigitalInput testInput{6};

  //create compressor
  frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};


//Default PID coefficientss
double shotRPM = 2500;//NOTE: The rpm drops by ~500 when ball goes in. (Increase shot speed by 500rpm)
double proportionalPIDConstant = 0.0003;
double intergralPIDConstant = 0.000001;
double derivativePIDConstant = 0;
double intergralZonePIDConstant = 0;
double feedForwardPIDConstant = 0;
double kMaxOutput = 1.0;
double kMinOutput = -1.0;

//Align Variables
double horizontalOffset = 0;
double turnPosition = 0; //In degrees
double leftTurnGoal = 0;
double rightTurnGoal = 0;

//Boolean Variables
bool indexActive = false;
bool intakeActive = false;
bool shootActive = false;

double autoSteps = 0;

double steeringAdjust = 0;

void Robot::onShotRequest(double goalHeight){
  Limelight::toggleCamera(0);
  if (Limelight::visibleTarget()){
    double goalDistance = Limelight::getDistance();

    steeringAdjust = Shooter::alignTarget();

    // double shotVelocity = Physics::getVelocity(goalDistance, goalHeight);
    // shotRPM = Physics::getShotRPM(shotVelocity);

    shotRPM = 2750;
    
    horizontalOffset = Limelight::getInfo("tx");
    
    frc::SmartDashboard::PutNumber("Steering Adjust", 0);
    std::cout << steeringAdjust << std::endl;

    if (shotRPM > 0 && Physics::farEnough(goalDistance, goalHeight) && abs(horizontalOffset) < alignThreshold) {
      frc::SmartDashboard::PutNumber("Shot RPM", shotRPM);
      std::cout << shotRPM << std::endl;

      if (abs(goalDistance - shotDistance) > distanceThreshold) {
        if (goalDistance > shotDistance) {
          m_robotDrive.ArcadeDrive(0, minDrivePower); 
        } else if (goalDistance < shotDistance) {
          m_robotDrive.ArcadeDrive(0, -minDrivePower);
        }
      }

      Shooter::shoot(m_pidController, shotRPM);
      double currentRPM = m_encoder.GetVelocity();
      bool speedReached = currentRPM > (shotRPM - rpmThreshold) && currentRPM < (shotRPM + rpmThreshold);
      if (speedReached) {
        indexActive = true;
      }
    } else {
      m_robotDrive.ArcadeDrive(steeringAdjust, 0);
    }
  }
}

  //Create a mode toggle for manual climb control and a mode toggle for intake direction
  bool buttonPressed = false;
  bool testBool = false;
  bool intake;
  bool climbing;

  climb climbObject;

  //Previous accelerometer values
  double prevXAccel = 0;
  double prevYAccel = 0;

  //frc::SendableChooser testChoices;


rev::CANSparkMax m_leftLeadingMotor{leadLeftSparkID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightLeadingMotor{leadRightSparkID, rev::CANSparkMax::MotorType::kBrushless};

frc::DifferentialDrive m_robotDrive{m_leftLeadingMotor, m_rightLeadingMotor};

//PID controller
rev::SparkMaxPIDController m_pidController = m_shootMotor.GetPIDController();
rev::SparkMaxRelativeEncoder m_encoder = m_shootMotor.GetEncoder();

//Default PID coefficientss
double proportionalPIDConstant = 0;
double intergralPIDConstant = 0;
double derivativePIDConstant = 0;
double intergralZonePIDConstant = 0;
double feedForwardPIDConstant = 0.000015;
double kMaxOutput = 1.0;
double kMinOutput = -1.0;

//Instantiate the buttons
frc::Joystick m_controlStick{controlJoystickID};
frc::Joystick m_driveStick{driveJoystickID};

void Robot::RobotInit() {

  frc::CameraServer::StartAutomaticCapture();
  
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);m_robotDrive.ArcadeDrive(m_controlStick.GetTwist(), -1 * m_controlStick.GetY());
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
  frc::SmartDashboard::PutNumber("Left Motor Postion", m_leftEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Right Motor Postion", m_rightEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Minimum Distance", Physics::getMinDistance(top_goal));

  //Custom debug class records debug log in /home/lvuser/DEBUG.txt
  //initialize and end are only run once and debug.out can be run as many times as you want.
  //debug.initialize("/home/lvuser/DEBUG.txt");
  //debug.out("test");
  //debug.end();
  dbg debug;

  //set climbing to false so that we can shoot
  climbing = false;
  intake = true;
  
  //Assign the PID variables
  m_pidController.SetP(proportionalPIDConstant);
  m_pidController.SetI(intergralPIDConstant);
  m_pidController.SetD(derivativePIDConstant);
  m_pidController.SetIZone(intergralZonePIDConstant);
  m_pidController.SetFF(feedForwardPIDConstant);
  m_pidController.SetOutputRange(kMinOutput, kMaxOutput);
  
  //Update the variables to the smartdashboard
  frc::SmartDashboard::PutNumber("P Gain", proportionalPIDConstant);
  frc::SmartDashboard::PutNumber("I Gain", intergralPIDConstant);
  frc::SmartDashboard::PutNumber("D Gain", derivativePIDConstant);
  frc::SmartDashboard::PutNumber("I Zone", intergralZonePIDConstant);
  frc::SmartDashboard::PutNumber("Feed Forward", feedForwardPIDConstant);
  frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
  frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
}
  
  
/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

  cs::CvSink cvSink = frc::CameraServer::GetVideo();

  cs::CvSource outputStream = frc::CameraServer::PutVideo("Bottom_Of_Bot", 480, 480);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */

void Robot::AutonomousInit() {
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

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
    m_robotControl.drive(-0.1, 0);
  } else {

    m_robotControl.drive(-0.1, 0);

  }
}

void Robot::TeleopInit() {
  //Shooter speed is set so we don't start with a spinning motor.
  shooterSpeed = 0.0;
  frc::SmartDashboard::PutNumber("Shooter Speed", shooterSpeed);
}

void Robot::DisabledPeriodic() {


void onShotRequest(double goal_height){
  Limelight::toggleCamera(0);
  double shot_rpm = Shooter::findTarget(goal_height);
  if (shot_rpm > 0) {

    Shooter::shoot(m_pidController, m_indexMotor, shot_rpm);
    if (m_encoder.GetVelocity() > (shot_rpm - rpmThreshold)) {
      m_indexMotor.Set(0.5);
    } else {
      m_indexMotor.Set(0);
    }
  }
}

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
  frc::SmartDashboard::PutNumber("Left Motor Postion", m_leftEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Right Motor Postion", m_rightEncoder.GetPosition());

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

  //Shooting
  if (m_controlStick.GetRawButton(shootHighButtonID)){
    std::cout << "Shoot High" << std::endl;
    Robot::onShotRequest(top_goal);

  } else if (m_controlStick.GetRawButton(shootLowButtonID)){    
    std::cout << "Shoot Low" << std::endl;
    Robot::onShotRequest(low_goal);

  } else {
    //Control Stick Drive Robot
    if (abs(m_driveStick.GetTwist()) >= joyStickDeadzone || abs(m_driveStick.GetY()) >= joyStickDeadzone) {
      m_robotDrive.ArcadeDrive(m_driveStick.GetTwist(), -m_driveStick.GetY());
    } else if (abs(m_controlStick.GetTwist()) >= joyStickDeadzone || abs(m_controlStick.GetY()) >= joyStickDeadzone) {
      m_robotDrive.ArcadeDrive(m_controlStick.GetTwist()*controlStickSensitivty, -m_controlStick.GetY()*controlStickSensitivty);
    }

    indexActive = false;
    shootActive = false;
    Shooter::shoot(m_pidController, 0);
    // Limelight::toggleCamera(1);
  }
  

  //Limelight
  if (m_controlStick.GetRawButtonPressed(cameraButtonID)){
    Limelight::toggleCamera();
  }

  //Index
  if (m_controlStick.GetRawButtonPressed(indexButtonID)) {
    indexActive = !indexActive;
  }

  if (indexActive) {
    m_indexMotor.Set(indexMotorSpeed);
  } else {
    m_indexMotor.Set(0.0);
  }

  //Intake
  if (m_controlStick.GetRawButtonPressed(intakeButtonID)) {
    intakeActive = !intakeActive;
  }

  if (intakeActive) {
    m_intakeMotor.Set(intakeMotorSpeed);
  } else {
    m_intakeMotor.Set(0.0);
  }

  double ve = Physics::getShotRPM(Physics::getVelocity(300, top_goal));

  // std::cout << ve << std::endl;
}


  //climber.prepareClimb(climbing);

//NOT-YET-TESTED tankdrive class.  Easily replacible using frc::arcadedrive(might not be exaxt function call).
  if (m_controlStick.GetY()>=0.1 || m_controlStick.GetY()<=-0.1 || m_controlStick.GetTwist()>=0.1 || m_controlStick.GetTwist()<=-0.1) {
    m_robotDrive.ArcadeDrive(m_controlStick.GetTwist(),-1 * m_controlStick.GetY());
  } 
//Toggles intake to reject balls
  if (m_controlStick.GetRawButtonReleased(8)) {
    intake = !intake;
  }
  if (intake == true) {
    m_intakeMotor.Set(1);
  } else {
    m_intakeMotor.Set(0);
  }
  if (m_climbStick.GetRawButtonPressed(8)) {
    climbObject.prepareClimb(&climbing);
  }

  if (climbing == true) {
    climbObject.doClimb(&m_climbStick);
  }
  

  //Sets shooter speed with (hopefully) working limits and output to smart dashboard.
  /*if (m_climbStick.GetRawButtonPressed(3) && shooterSpeed != 0.0 && climbing == false) {
    shooterSpeed = shooterSpeed - 0.1; 
    frc::SmartDashboard::PutNumber("Shooter Speed", shooterSpeed);
  }
  if (m_climbStick.GetRawButtonPressed(4) && shooterSpeed != 1.0 && climbing == false) {
    shooterSpeed = shooterSpeed + 0.1;
    frc::SmartDashboard::PutNumber("Shooter Speed", shooterSpeed);
  } */
  m_shooterMotor.Set(shooterSpeed);
  if (m_controlStick.GetRawButton(4)) {
    m_indexBelt.Set(1.0);
  } else {
    m_indexBelt.Set(0.0);
  }
}


  // read PID coefficients from SmartDashboard
  double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  double mx = frc::SmartDashboard::GetNumber("Max Output", 0);
  double mn = frc::SmartDashboard::GetNumber("Min Output", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
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

  if (m_controlStick.GetRawButton(shootHighButtonID)){
    onShotRequest(top_goal);
  } else if (m_controlStick.GetRawButton(shootLowButtonID)){    
    onShotRequest(low_goal);
  }

  if (m_controlStick.GetRawButtonPressed(shootLowButtonID) || m_controlStick.GetRawButtonPressed(shootHighButtonID)){
    Shooter::alignTarget(m_robotDrive);
  }

  if (m_controlStick.GetRawButtonReleased(shootLowButtonID) || m_controlStick.GetRawButtonReleased(shootHighButtonID)){
    Shooter::shoot(m_pidController, m_indexMotor, 0);
    m_indexMotor.Set(0);
    Limelight::toggleCamera(1);
  }

  if (m_controlStick.GetRawButtonPressed(cameraButtonID)){
    Limelight::toggleCamera();
  }
}

void Robot::TestPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
