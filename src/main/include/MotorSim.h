#pragma once
#include "Constants.h"

/**
 * A class that acts as a basic simulation for a motor, motor controller, and encoder to be used in this test
**/

class MotorSim {
    public:
        void periodic();
        void setVoltage(double volts);
        double getAngleRads();
        double getAngVelRads();
        void setAngleRads(double newAngle);
        void setAngleVelRads(double newVel);

    private:
        double angle_ = 0;
        double angVel_ = 0;
        double voltage_ = 0;
};