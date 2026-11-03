#include "main.h"
#include "robot.h"
#include "mcl.h"
#include "angled_odom.h"
#include "telemetry.h"

void initialize() {
    pros::lcd::initialize();
	imu.reset(true);
	angledOdom::configure(&back_left_encoder, &back_right_encoder, &imu, &chassis, 2, -1.5945, 1.5945);
	angledOdom::start(48, 0, 0);
	static telemetry mclLog("/usd/telemetry.csv", 300, 4, MCL::LOOP_MS);
	MCL::setTelemetry(&mclLog);
	MCL::init(300);

	pros::Task screenTask([] {
		while (true) {
			pros::lcd::print(4, "E1: %d E2: %d", back_left_encoder.get_value(), back_right_encoder.get_value());
			pros::lcd::print(5, "IMU: %.2f", imu.get_rotation());
			lemlib::Pose pose = angledOdom::getPose();
			pros::lcd::print(1, "X: %.2f", pose.x);
			pros::lcd::print(2, "Y: %.2f", pose.y);
			pros::lcd::print(3, "Theta: %.2f", pose.theta);
			pros::delay(50);
		}
	}, "screen");

}

void disabled() {}

void competition_initialize() {}

// Average actual velocity (rpm) across every motor in the group
static double avgVelocity(const pros::MotorGroup& mg) {
	std::vector<double> v = mg.get_actual_velocity_all();
	double sum = 0;
	for (double rpm : v) sum += rpm;
	return v.empty() ? 0 : sum / v.size();
}

void autonomous() {
	// Brake instead of coasting when LemLib sets the motors to 0
	//left_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	//right_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	chassis.setPose(0,0,0); // -24 -24 0 
	pros::delay(500);
	chassis.moveToPoint(0, 30, 4000);
	chassis.turnToHeading(180, 3000);

	// Target setpoint (the destination we want, e.g., 24 inches)
	const double setpoint = 48.0;
	// Keep logging for 1.5s after the motion ends so any slide shows up on the plot
	uint32_t settleEnd = 0;
	while (chassis.isInMotion() || pros::millis() < settleEnd) {
		if (!chassis.isInMotion() && settleEnd == 0) settleEnd = pros::millis() + 1500;

		lemlib::Pose pose = chassis.getPose();
		double error = setpoint - pose.y;

		// Print the data strictly formatted for the serial plotter
		// This MUST be on a single line ending with \n
		printf("Setpoint:%.2f,Y:%.2f,Error:%.2f,X:%.2f,Theta:%.2f,LeftRPM:%.1f,RightRPM:%.1f\n",
		       setpoint, pose.y, error, pose.x, pose.theta, avgVelocity(left_mg), avgVelocity(right_mg));

		// 50ms so the controller's wireless link can keep up (10ms matches LemLib's loop if tethered over USB)
		pros::delay(50);
	}
}