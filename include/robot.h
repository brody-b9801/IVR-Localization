#pragma once
#include "api.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

extern pros::Controller master;

extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;

extern pros::Imu imu;

extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

extern pros::Distance distanceFront;
extern pros::Distance distanceLeft;
extern pros::Distance distanceRight;
extern pros::Distance distanceBack;
