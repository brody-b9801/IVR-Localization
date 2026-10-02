#pragma once
#include <iostream>
#include <vector>
#include "api.h"
#include "utilities.hpp"
#include "robot.h"

using namespace std;
using namespace utilities;

class fieldRepresentation {
    public:
        struct distanceSensorDistances { // inches, -1 if nothing in range
            double frontDistance;
            double leftDistance;
            double rightDistance;
            double backDistance;
            bool updated;
        };

    private:
        struct fieldObject {
            string type;
            vector3 min;
            vector3 max;
        };

        static constexpr double maxSensorRange = 78.74; // V5 distance sensor max range, 2000mm in inches

        double hitDistance(const fieldObject& object, vector3 sensorPos, vector3 sensorDir); //return -1 if not hit
        double closestHit(vector3 sensorPos, vector3 sensorDir); //return -1 if nothing in range

        static vector<fieldObject> fieldObjects();
        static vector<distanceSensor> distanceSensors();

        vector<fieldObject> objects_;
        vector<distanceSensor> sensors_;

    public:
        fieldRepresentation(vector<distanceSensor> sensors); // uses fieldObjects()
        fieldRepresentation(vector<distanceSensor> sensors, vector<fieldObject> objects);
        distanceSensorDistances simulateCast(vector3 particlePos, double particleHeading); // particleHeading = imu heading + particle's heading offset
        vector3 getMax(const fieldObject& object) { return object.max; }
        vector3 getMin(const fieldObject& object) { return object.min; }
};
