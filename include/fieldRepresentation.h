#pragma once
#include <iostream>
#include <vector>
#include "api.h"
#include "utilities.hpp"
#include "robot.h"

using namespace std;
using namespace utilities;

class fieldRepresentation {
    private:
        struct distanceSensorDistances {
            double frontDistance;
            double leftDistance;
            double rightDistance;
            double backDistance;
            bool updated;
        };

        struct fieldObject {
            string type;
            vector3 min;
            vector3 max;
        };

        struct distanceSensor {
            vector3 direction;
            pros::Distance sensor;
            vector3 centerOffset;
        };

        double hitDistance(fieldObject object, distanceSensor sensor, vector3 robotPosition, double robotHeading); //return -1 if not hit

        static vector<fieldObject> fieldObjects();
        static vector<distanceSensor> distanceSensors();

        vector<fieldObject> objects_;
        vector<distanceSensor> sensors_;

    public:
        fieldRepresentation(vector<distanceSensor> sensors); // uses fieldObjects()
        fieldRepresentation(vector<distanceSensor> sensors, vector<fieldObject> objects);
        distanceSensorDistances simulateCast(distanceSensor sensor, vector3 castDirection, vector3 robotPosition);

};
