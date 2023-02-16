#pragma once

#include <frc2/command/TrapezoidProfileSubsystem.h>
#include <units/voltage.h>

class ProfileSubsystem : frc2::TrapezoidProfileSubsystem<units::radians> {
    
    using TrapezoidProfile = frc::TrapezoidProfile<units::radians>;
    using State = frc::TrapezoidProfile<units::radians>::State;

    public:
        ProfileSubsystem(TrapezoidProfile::Constraints constraints);

        void update();
        void UseState(State state) override;
        void SetGoalState(State state);
        State getProfile(); //gets the profile at the current time

    private:
        State currProfile_;
};