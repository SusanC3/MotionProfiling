// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "Constants.h"
#include "ProfileSubsystem.h"
#include <units/angular_velocity.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/PIDController.h>
#include "MotorSim.h"
#include <algorithm>
#include <fstream>
#include <frc/Timer.h>
#include <iostream>

using TrapezoidProfile = frc::TrapezoidProfile<units::radians>;


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:

 MotorSim motor_;

  //WPILib trapezoidal motion + PID + feedforward
  TrapezoidProfile::Constraints constraints{
      TrapezoidProfile::Velocity_t{TestConstants::MAX_VEL}, TrapezoidProfile::Acceleration_t{TestConstants::MAX_ACCEL}};
  ProfileSubsystem trapProfileSubsyst_{constraints};

  frc::SimpleMotorFeedforward<units::radians> feedforward_{TestConstants::kS, TestConstants::kV, TestConstants::kA};

  frc::PIDController PID_{TestConstants::kP, 0, TestConstants::kD}; //set kI to 0 to because TrajectoryCalc doesn't include kI

  
};
