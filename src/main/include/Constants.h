#pragma once

#include <units/voltage.h>
#include <units/angle.h>
#include <units/time.h>


namespace GeneralConstants {
    const double MAX_VOLTAGE = 12;
}

namespace TestConstants {
	const double MAX_VEL = 100;
	const double MAX_ACCEL = 1;
	const double kP = 10;
	const double kD = 0;
	const auto kS = 0.24387_V; //otherwise known as kI
    const auto kV = 2.1675 * 1_V * 1_s / 1_rad;
    const auto kA = 0.27807 * 1_V * 1_s * 1_s / 1_rad;
}