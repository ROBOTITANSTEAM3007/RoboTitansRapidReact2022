#pragma once

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
#include <fstream>
//#include"Debug.h"

class climb {
    public: 
        void doClimb(frc::Joystick* joy);
        void prepareClimb(bool* climbingBool);
        void doManualClimb(frc::Joystick* joy);
        void climbTest(frc::Joystick* joy);
        bool armGo = false;
        dbg debug;


    private:
        frc::DigitalInput handOne{1};
        frc::DigitalInput handTwo{0};
        unsigned short grabs = 0;
        bool whichSwitch = true;
        bool lastHandOne = false;
        bool lastHandTwo = false;
        long long firstIterations = 0;
        long long secondIterations = 0;

        static unsigned const short climbingMotorID = 4;
        rev::CANSparkMax m_climbingMotor{climbingMotorID, rev::CANSparkMax::MotorType::kBrushless};

                rev::SparkMaxRelativeEncoder m_climbEncoder = m_climbingMotor.GetEncoder();
        
        //Instantiate a group solenoids for pneumatic control
        frc::Solenoid m_grab1{frc::PneumaticsModuleType::CTREPCM, 0};
        frc::Solenoid m_grab2{frc::PneumaticsModuleType::CTREPCM, 3};

        frc::Solenoid m_armRelease{frc::PneumaticsModuleType::CTREPCM, 2};
};