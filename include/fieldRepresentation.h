#pragma once
#include <iostream>
#include <vector>
#include "api.h"
#include "utilities.h"

using namespace std;
using namespace utilities;

class fieldRepresentation {
    public:
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
            string direction;
            pros::Distance sensor;
            vector3 centerOffset;
        };

    private:
        double hitDistance(fieldObject); //return -1 if not hit

        // static objects on the field, from Resources/field-model/field_boxes.json (floor excluded)
        static vector<fieldObject> fieldObjects();

        vector<fieldObject> objects_;
        vector<distanceSensor> sensors_;

    public:
        fieldRepresentation(vector<distanceSensor> sensors); // uses fieldObjects()
        fieldRepresentation(vector<distanceSensor> sensors, vector<fieldObject> objects);
        distanceSensorDistances simulateCast(distanceSensor sensor, vector3 castDirection, vector3 robotPosition);

};
