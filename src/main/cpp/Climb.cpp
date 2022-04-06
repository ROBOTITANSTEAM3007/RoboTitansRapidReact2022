#include "Robot.h"
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include <frc/drive/DifferentialDrive.h>
#include <iostream>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Compressor.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include "Climb.h"
#include <fstream>
#include "Debug.h"


void climb::prepareClimb(bool* climbingBool) {

    m_grab1.Set(true);
    m_grab2.Set(true);
    m_armRelease.Set(false);

    frc::SmartDashboard::PutBoolean("Arm One Is Grabbed", m_grab1.Get());
    frc::SmartDashboard::PutBoolean("Arm Two Is Grabbed", m_grab2.Get());

    //MUST come at the end
    m_armRelease.Toggle();
    *climbingBool = true;
}

//Must be used in TeleopPeriodic()


//Need to make driver for lamprey absulute encoder
void climb::doClimb(frc::Joystick* joy) {
    frc::SmartDashboard::PutNumber("Climb Encoder", m_climbEncoder.GetPosition());
    if (joy->GetRawButton(1)) {
        m_climbingMotor.Set(joy->GetY() * 0.5);
    } else {
        m_climbingMotor.Set(0);
    }
    if(joy->GetRawButtonPressed(2)) {
        m_armRelease.Toggle();

    }
    if(joy->GetRawButtonPressed(5)){
        m_grab1.Toggle();
    }
    if(joy->GetRawButtonPressed(6)) {
        m_grab2.Toggle();
    }
    if (!handOne.Get()) {
        debug.out("Hand One Button Has Been Pressed");
    }
    if (!handTwo.Get()) {
        debug.out("Hand Two Button Has Been Pressed");
    }
    if (firstIterations <=10 && !handOne.Get()) {
        firstIterations++;
    } else {
        firstIterations = 0;
    }
    if (secondIterations <=10 && !handTwo.Get()) {
        secondIterations++;
    } else {
        secondIterations = 0;
    }
    if (!handOne.Get() && !lastHandOne && firstIterations > 10){
        m_grab1.Set(false);
        debug.out("Hand One has grabbed");
        lastHandOne = !handOne.Get();
    } else {
        lastHandOne = !handOne.Get();  
    }
    if (!handTwo.Get() && !lastHandTwo && secondIterations > 10) {
        m_grab2.Set(false);
        debug.out("Hand Two has grabbed");
        lastHandTwo = !handTwo.Get();
    } else {
        lastHandTwo = !handTwo.Get();
    }
    if (grabs < 3) {
        if (joy->GetRawButtonPressed(3) && !m_grab2.Get() && !m_grab1.Get() && !handOne.Get() && !handTwo.Get()) {
            if(whichSwitch) {
                m_grab2.Set(true);
                whichSwitch = !whichSwitch;
                grabs++;
            } else {
                m_grab1.Set(true);
                whichSwitch = !whichSwitch;
                grabs++;
            }
        }
    } else {
        debug.end();
    }
    if (grabs < 3 || joy->GetRawButtonPressed(12)) {
        debug.end();
        m_grab1.Set(false);
        m_grab2.Set(false);
    }

    frc::SmartDashboard::PutBoolean("Arm One Is Grabbed", !m_grab1.Get());
    frc::SmartDashboard::PutBoolean("Arm Two Is Grabbed", !m_grab2.Get());
}

void climb::doManualClimb(frc::Joystick* joy) {
    frc::SmartDashboard::PutNumber("Climb Encoder", m_climbEncoder.GetPosition());
    if (joy->GetRawButtonPressed(3)) {
        m_armRelease.Toggle();
    }
    if (joy->GetRawButtonPressed(4) && armGo == false) {
        armGo = true;
        m_climbingMotor.Set(-0.1);
    }
    if (joy->GetRawButtonPressed(4) && armGo == true) {
        armGo = false;
        m_climbingMotor.Set(0.0);
    }
    if (joy->GetRawButtonPressed(5)) {
        m_grab1.Toggle();
    }
    if (joy->GetRawButtonPressed(6)) {
        m_grab2.Toggle();
    }

    frc::SmartDashboard::PutBoolean("Arm One Is Grabbed", !m_grab1.Get());
    frc::SmartDashboard::PutBoolean("Arm Two Is Grabbed", !m_grab2.Get());

}
void climb::climbTest(frc::Joystick* joy) {

    frc::SmartDashboard::PutNumber("Climb Encoder", m_climbEncoder.GetPosition());
    if (joy->GetRawButton(1)) {
        m_climbingMotor.Set(joy->GetY() * 0.5);
    } else {
        m_climbingMotor.Set(0);
    }
   /* if (joy->GetRawButton(1) {
        armGo = false;
        m_climbingMotor.Set(0.0);
    }*/
    if(joy->GetRawButtonPressed(2)) {
        m_armRelease.Toggle();
    }
    if(joy->GetRawButtonPressed(5)){
        m_grab1.Toggle();
    }
    if(joy->GetRawButtonPressed(6)) {
        m_grab2.Toggle();
    }

    if (!handOne.Get() && !lastHandOne){
        m_grab1.Set(false);
        lastHandOne = !handOne.Get();
    } else {
        lastHandOne = !handOne.Get();  
    }
    if (!handTwo.Get() && !lastHandTwo) {
        m_grab2.Set(false);
        lastHandTwo = !handTwo.Get();
    } else {
        lastHandTwo = !handTwo.Get();
    }
    if (joy->GetRawButtonPressed(3) && !m_grab2.Get() && !m_grab1.Get() && !handOne.Get() && !handTwo.Get()) {
        if(whichSwitch) {
            m_grab2.Set(true);
            whichSwitch = !whichSwitch;
            grabs++;
        } else {
            m_grab1.Set(true);
            whichSwitch = !whichSwitch;
            grabs++;
        }
    }
    frc::SmartDashboard::PutBoolean("Arm One Is Grabbed", !m_grab1.Get());
    frc::SmartDashboard::PutBoolean("Arm Two Is Grabbed", !m_grab2.Get());

}