#include "Limelight.h"
#include <iostream>

auto Limelight::getTable(){//Returns the network table instance.
    auto network_instance = nt::NetworkTableInstance::GetDefault();
    auto variable_table = network_instance.GetTable("Preferences");
    auto cam_table = network_instance.GetTable("limelight");

    return cam_table;
};

double Limelight::getInfo(const std::string& name){//Gets the information from the network tables and returns it.
    double value = Limelight::getTable()->GetNumber(name, 0.0);

    return value;
};

void Limelight::putInfo(const std::string& name, double value){//Sets the information to the network table.
    Limelight::getTable()->PutNumber(name, value);
};

void Limelight::toggleCamera(){//Turns on/off the limelight and camera.
    int current_cam = Limelight::getInfo("camMode"); // 0 tracking, 1 driving vision.
    int current_led = Limelight::getInfo("ledMode"); // 1 off, 3 on.

    Limelight::putInfo("camMode", (current_cam + 1) % 2);

    if (current_led == 3){
        Limelight::putInfo("ledMode", 1);
    } else {
        Limelight::putInfo("ledMode", 3);
    }
};

void Limelight::toggleCamera(int mode){//0 is automatic, 1 is driver.
    if (mode == 0) {
        Limelight::putInfo("ledMode", 3);
        Limelight::putInfo("camMode", 0);
    } else {
        Limelight::putInfo("ledMode", 1);
        Limelight::putInfo("camMode", 1);
    }
};

bool Limelight::visibleTarget(){//Returns if the target is visible

    int target_visible = Limelight::getInfo("tv") == 1;//1 and 0

    if (!target_visible){
        std::cout << "Target Not Visible!" << std::endl;
    };

    frc::SmartDashboard::PutBoolean("Target Visible", target_visible);

    return target_visible;
    // return true;

};

double Limelight::getDistance(){//Calculates the horizontal distance with the limelight variables.
    double vertical_angle_offset = Limelight::getInfo("ty");//-24.85 to +24.85 degrees

    double height = top_goal - cam_height;//The difference of height between the goal and the camera.(The leg of the right triangle.)
    double angle = cam_angle + (vertical_angle_offset * (PI/180));//Total angle of the mount and camera in degrees.(The angle of the right triangle.)

    double distance = height/tan(angle) + offset_distance;//Horizontal distance from the goal to the shooter

    return distance;
};