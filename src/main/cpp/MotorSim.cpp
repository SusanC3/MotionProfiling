#include "MotorSim.h"

/**
 * A class that acts as a basic simulation for a motor, motor controller, and encoder to be used in this test
**/

/**
 * Use simplified voltage-balance equation to simulate motor behavior
 * https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/introduction.html#simple-motor-identification
**/
void MotorSim::periodic() {
    //Volts = kV*velocity + kS --> velocity = (volts - kS)/kV
    angVel_ = (voltage_ - TestConstants::kS.value())/TestConstants::kV.value();
    angle_ += angVel_*0.02; //assuming period = 20ms
}

void MotorSim::setVoltage(double volts) {
    voltage_ = volts;
}

double MotorSim::getAngleRads() {
    return angle_;
}

void MotorSim::setAngleRads(double newAngle) {
    angle_ = newAngle;
}

double MotorSim::getAngVelRads() {
    return angVel_;
}

void MotorSim::setAngleVelRads(double newVel) {
    angVel_ = newVel;
}