#pragma once

#include "api.h"
using namespace pros;

class Intake {
    public:
        /**
         * Constructor
         *
         * @param port intake motor port
         */
        Intake(int port) : intakeMotor(port, v5::MotorGear::blue) {};

        /**
         * Spins the intake @ speed until called again (move(0) to stop)
         *
         * @param speed -127 to 127
         */
        void move(int speed) { intakeMotor.move(speed); };

    private:
        Motor intakeMotor;
};