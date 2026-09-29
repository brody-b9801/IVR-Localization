#pragma once
#include "api.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

extern pros::Controller master;
extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern lemlib::Drivetrain drivetrain;
extern pros::Imu imu;
extern lemlib::Chassis chassis;