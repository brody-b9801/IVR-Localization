#pragma once
#include "fieldRepresentation.h"
#include "robot.h"
#include <vector>
#include "utilities.hpp"
#include "particle.h"

using namespace std;

class MCL {
    private:
        static fieldRepresentation::distanceSensorDistances sensorData_;
        static vector<particle> particles_;
        static lemlib::Pose prevPose;
        static fieldRepresentation& getField();
        static fieldRepresentation::distanceSensorDistances readSensors();
        static double effectiveParticleCount();
    public:
        static void setSensorData(fieldRepresentation::distanceSensorDistances data) { sensorData_ = data; }

        static void init(int numParticles);
        static void update(double forward, double right, double imuHeading); // robot-frame odometry change, inches
        static lemlib::Pose weigh(fieldRepresentation& field);
        static void resample();

};