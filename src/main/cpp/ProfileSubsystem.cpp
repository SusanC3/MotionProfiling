#include "ProfileSubsystem.h"

//profile subsystem is a TrapezoidProfileSubsystem
ProfileSubsystem::ProfileSubsystem(TrapezoidProfile::Constraints constraints) : frc2::TrapezoidProfileSubsystem<units::radians>(constraints) {
    this->Enable(); //so that useState will be called every 20ms
}

//this function will be called periodically in the background
void ProfileSubsystem::UseState(State state) {
    currProfile_ = state;
}

void ProfileSubsystem::SetGoalState(State state) {
    this->SetGoal(state);
}

ProfileSubsystem::State ProfileSubsystem::getProfile() {
    return currProfile_;
}

void ProfileSubsystem::update() {
    this->Periodic();
}