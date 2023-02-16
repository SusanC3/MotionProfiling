// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//https://www.desmos.com/calculator/nlaovarrnj

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <filesystem>

void Robot::RobotInit() {
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() { 
  //starts at 1 rad with 0 vel, goal is angle of 2 radian and velocity of 0 rad/s
  trapProfileSubsyst_.SetGoalState({units::radian_t{1}, units::radians_per_second_t{0}}); //desired goal distance is 1 away 
  motor_.setAngleRads(1);
}

void Robot::TeleopPeriodic() {
  trapProfileSubsyst_.update();

  //motors update their physical states
  motor_.periodic();

  //calculate current profile/goal point
  TrapezoidProfile::State profile= trapProfileSubsyst_.getProfile();

  //calculate output voltage to achieve goal 
  double volts = std::clamp( //restricts voltage output to be within the min and max voltage
    feedforward_.Calculate(profile.velocity).value() + PID_.Calculate(motor_.getAngleRads(), profile.position.value()), 
    -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE); 

  //set motor voltage
  motor_.setVoltage(volts);
  
  std::cout << "profile: pose = " << profile.position.value()<< ", motor pose = " << motor_.getAngleRads() << "\n";
  std::cout << "PID volts: " << PID_.Calculate(motor_.getAngleRads(), profile.position.value()) << "\n";
  std::cout << "voltsWPILib: " << volts << "\n";
  std::cout << "\n";  

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
