#pragma once
#include "fieldRepresentation.h"
#include "robot.h"
#include <vector>
#include "utilities.hpp"

using namespace std;

class MCL {
    private:
        static fieldRepresentation::distanceSensorDistances sensorData_;
    public:
        static void setSensorData(fieldRepresentation::distanceSensorDistances data) { sensorData_ = data; }


};