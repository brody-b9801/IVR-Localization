#pragma once

#include "api.h"
#include <cmath>
using namespace pros;

class Cascade {
    public:
        /**
         * Constructor
         *
         * @param motorPort1 first cascade motor port 
         * @param motorPort2 second cascade motor port 
         * @param rotationPort rotation sensor port
         * @param pulleyDiameter diameter of pulley in inches
         * @param numStages number of stages in the cascade
         */
        Cascade(std::int8_t motorPort1, std::int8_t motorPort2, std::int8_t rotationPort, double pulleyDiameter, int numStages)
            : cascadeMotors({motorPort1, motorPort2}), rotationSensor(rotationPort), spoolDiameter(pulleyDiameter), stages(numStages) {
            cascadeMotors.set_brake_mode(MotorBrake::hold);
        };

        /**
         * Moves the cascade @ speed until called again (move(0) to stop)
         *
         * @param speed -127 to 127
         */
        void move(int speed) { cascadeMotors.move(speed); };

        /**
         * Moves the cascade to a height
         *
         * @param inches target height in inches
         */
        void SetHeight(double inches);

    private:
        double targetHeight2Deg(double inches);
        MotorGroup cascadeMotors;
        Rotation rotationSensor;
        double spoolDiameter;
        int stages;
};
