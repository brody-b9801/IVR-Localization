#include "robot.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-1, 2, 3, -4}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({-5, 6, 7, -8}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&left_mg,
                              &right_mg,
                              10, // track width
                              2.5, // wheel size
                              360, // drivetrain rpm
                              2 // horizontal drift
);

pros::Imu imu(10);
pros::adi::Encoder horizontal_encoder('A', 'B', true);
pros::adi::Encoder vertical_encoder('C', 'D', true);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

//Placeholders
pros::Distance distanceFront(10);
pros::Distance distanceLeft(11);
pros::Distance distanceRight(12);
pros::Distance distanceBack(13);

lemlib::OdomSensors sensors(&vertical_tracking_wheel,
                            nullptr,
                            &horizontal_tracking_wheel, 
                            nullptr,
                            &imu 
);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors
);