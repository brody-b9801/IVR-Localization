#pragma once

#include "api.h"
using namespace pros;

class Claw {
    public:
        /**
         * Constructor
         *
         * @param actuatorPort actuator solenoid ADI port
         * @param actuatorFlipped true if the actuator extends on a LOW signal
         * @param clampPort clamp solenoid ADI port
         * @param clampFlipped true if the clamp extends on a LOW signal
         * @param wristPort wrist solenoid ADI port
         * @param wristFlipped true if the wrist extends on a LOW signal
         */
        Claw(char actuatorPort, bool actuatorFlipped, char clampPort, bool clampFlipped, char wristPort, bool wristFlipped)
            : actuator_solenoid(actuatorPort, false, actuatorFlipped), clamp_solenoid(clampPort, false, clampFlipped), wrist_solenoid(wristPort, false, wristFlipped) {};

        /** Moves the wrist to its normal position */
        void setWristNormal() { wrist_solenoid.retract(); };

        /** Moves the wrist to its flipped position */
        void setWristFlipped() { wrist_solenoid.extend(); };

        /** Switches the wrist between normal and flipped */
        void toggleWrist() { wrist_solenoid.toggle(); };

        /** Moves claw matchload/score */
        void setActuatorMatchload() { actuator_solenoid.extend(); };

        /** Moves claw to intake */
        void setActuatorIntake() { actuator_solenoid.retract(); };

        /** Switches claw between matchload and intake */
        void toggleActuator() { actuator_solenoid.toggle(); };

        /** Closes the clamp */
        void setClampClosed() { clamp_solenoid.extend(); };

        /** Opens the clamp */
        void setClampOpen() { clamp_solenoid.retract(); };

        /** Switches the clamp between closed and open */
        void toggleClamp() { clamp_solenoid.toggle(); };

    private:
        adi::Pneumatics actuator_solenoid;
        adi::Pneumatics clamp_solenoid;
        adi::Pneumatics wrist_solenoid;
};
