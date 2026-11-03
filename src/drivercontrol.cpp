#include "main.h"
#include "robot.h"

using Axis = pros::controller_analog_e_t;
using Button = pros::controller_digital_e_t;

enum class DriveMode { Arcade, Tank };

struct DriverProfile {
    const char* name;
    DriveMode driveMode;
    Axis throttle;
    Axis turn;
    Axis tankLeft;
    Axis tankRight;
    Button intakeIn;
    Button intakeOut;
    Button cascadeUp;
    Button cascadeDown;
    Button clampToggle;
    Button wristToggle;
    Button actuatorToggle;
    int intakeSpeed;
    int cascadeSpeed;
};

static const DriverProfile profiles[] = {
    {
        .name = "Default",
        .driveMode = DriveMode::Arcade,
        .throttle = pros::E_CONTROLLER_ANALOG_LEFT_Y,
        .turn = pros::E_CONTROLLER_ANALOG_RIGHT_X,
        .tankLeft = pros::E_CONTROLLER_ANALOG_LEFT_Y,
        .tankRight = pros::E_CONTROLLER_ANALOG_RIGHT_Y,
        .intakeIn = pros::E_CONTROLLER_DIGITAL_R1,
        .intakeOut = pros::E_CONTROLLER_DIGITAL_R2,
        .cascadeUp = pros::E_CONTROLLER_DIGITAL_L1,
        .cascadeDown = pros::E_CONTROLLER_DIGITAL_L2,
        .clampToggle = pros::E_CONTROLLER_DIGITAL_A,
        .wristToggle = pros::E_CONTROLLER_DIGITAL_B,
        .actuatorToggle = pros::E_CONTROLLER_DIGITAL_X,
        .intakeSpeed = 127,
        .cascadeSpeed = 127,
    },
    {
        .name = "Tank",
        .driveMode = DriveMode::Tank,
        .throttle = pros::E_CONTROLLER_ANALOG_LEFT_Y,
        .turn = pros::E_CONTROLLER_ANALOG_RIGHT_X,
        .tankLeft = pros::E_CONTROLLER_ANALOG_LEFT_Y,
        .tankRight = pros::E_CONTROLLER_ANALOG_RIGHT_Y,
        .intakeIn = pros::E_CONTROLLER_DIGITAL_L1,
        .intakeOut = pros::E_CONTROLLER_DIGITAL_L2,
        .cascadeUp = pros::E_CONTROLLER_DIGITAL_R1,
        .cascadeDown = pros::E_CONTROLLER_DIGITAL_R2,
        .clampToggle = pros::E_CONTROLLER_DIGITAL_Y,
        .wristToggle = pros::E_CONTROLLER_DIGITAL_RIGHT,
        .actuatorToggle = pros::E_CONTROLLER_DIGITAL_DOWN,
        .intakeSpeed = 127,
        .cascadeSpeed = 100,
    },
};

static int activeProfile = 0;

static void drive(const DriverProfile& p) {
    if (p.driveMode == DriveMode::Tank) {
        left_mg.move(master.get_analog(p.tankLeft));
        right_mg.move(master.get_analog(p.tankRight));
        return;
    }
    int dir = master.get_analog(p.throttle);
    int turn = master.get_analog(p.turn);
    left_mg.move(dir - turn);
    right_mg.move(dir + turn);
}

static void runIntake(const DriverProfile& p) {
    if (master.get_digital(p.intakeIn)) intake.move(p.intakeSpeed);
    else if (master.get_digital(p.intakeOut)) intake.move(-p.intakeSpeed);
    else intake.move(0);
}

static void runCascade(const DriverProfile& p) {
    if (master.get_digital(p.cascadeUp)) cascade.move(p.cascadeSpeed);
    else if (master.get_digital(p.cascadeDown)) cascade.move(-p.cascadeSpeed);
    else cascade.move(0);
}

static void runClaw(const DriverProfile& p) {
    if (master.get_digital_new_press(p.clampToggle)) claw.toggleClamp();
    if (master.get_digital_new_press(p.wristToggle)) claw.toggleWrist();
    if (master.get_digital_new_press(p.actuatorToggle)) claw.toggleActuator();
}

void opcontrol() {
    left_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    master.set_text(0, 0, profiles[activeProfile].name);
    while (true) {
        const DriverProfile& p = profiles[activeProfile];
        drive(p);
        runIntake(p);
        runCascade(p);
        runClaw(p);
        pros::delay(20);
    }
}
