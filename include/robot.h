#pragma once
#include "api.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "intake.h"
#include "cascade.h"
#include "claw.h"

extern pros::Controller master;

extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;

extern pros::Imu imu;
extern pros::adi::Encoder back_left_encoder;
extern pros::adi::Encoder back_right_encoder;

extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

extern pros::Distance distanceFront;
extern pros::Distance distanceLeft;
extern pros::Distance distanceRight;
extern pros::Distance distanceBack;

extern Intake intake;
extern Cascade cascade;
extern Claw claw;

