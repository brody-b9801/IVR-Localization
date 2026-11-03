#include "fieldRepresentation.h"
#include <limits>

fieldRepresentation::fieldRepresentation() : fieldRepresentation(distanceSensors(), fieldObjects()) {}
fieldRepresentation::fieldRepresentation(vector<distanceSensor> sensors) : fieldRepresentation(sensors, fieldObjects()) {}
fieldRepresentation::fieldRepresentation(vector<distanceSensor> sensors, vector<fieldObject> objects) : objects_(objects), sensors_(sensors) {
    for (const distanceSensor& sensor : sensors_) {
        double sensorZ = sensor.getCenterOffset().z();
        vector<fieldObject> candidates;
        for (const fieldObject& object : objects_) {
            if (sensorZ >= object.min.z() && sensorZ <= object.max.z()) candidates.push_back(object);
        }
        sensorCandidates_.push_back(candidates);
    }
}

// Axis-aligned boxes in inches: origin at field centre on top of the foam tiles,
// +X towards the blue alliance station, +Z up. min/max are opposite corners.
vector<fieldRepresentation::fieldObject> fieldRepresentation::fieldObjects() {
    vector<fieldObject> objects;

    objects.push_back(fieldObject{"wall", vector3(-fieldHalfWidth, -fieldHalfWidth, -0.622), vector3(fieldHalfWidth, fieldHalfWidth, 11.5368), true});

    objects.push_back(fieldObject{"loader", vector3(66.4645, 56.346, 3.248), vector3(70.1968, 61.1804, 14.3701)}); // Blue loader tube
    objects.push_back(fieldObject{"loader", vector3(-70.1968, -61.1804, 3.5573), vector3(-66.4645, -56.346, 14.6793)}); // Red loader tube
    objects.push_back(fieldObject{"loader", vector3(66.4645, -61.1804, 3.248), vector3(70.1968, -56.346, 14.3701)}); // Blue loader tube
    objects.push_back(fieldObject{"loader", vector3(-70.1968, 56.346, 3.5573), vector3(-66.4645, 61.1804, 14.6793)}); // Red loader tube

    objects.push_back(fieldObject{"goal", vector3(-49.8988, 20.738, 0), vector3(-44.2836, 26.3532, 5.769)}); // Neutral Goal 1
    objects.push_back(fieldObject{"goal", vector3(-26.3532, 44.2836, 0), vector3(-20.738, 49.8988, 5.769)}); // Neutral Goal 4
    objects.push_back(fieldObject{"goal", vector3(44.2856, 20.74, 0), vector3(49.8968, 26.3512, 3.2493)}); // Blue 2
    objects.push_back(fieldObject{"goal", vector3(-49.8968, -26.3512, 0), vector3(-44.2856, -20.74, 3.2493)}); // Red 2
    objects.push_back(fieldObject{"goal", vector3(20.74, 44.2856, 0), vector3(26.3512, 49.8968, 3.2493)}); // Blue 3
    objects.push_back(fieldObject{"goal", vector3(-2.8076, -2.8076, 0), vector3(2.8076, 2.8076, 8.769)}); // Neutral Goal 0
    objects.push_back(fieldObject{"goal", vector3(-26.3512, -49.8968, 0), vector3(-20.74, -44.2856, 3.2493)}); // Red 3
    objects.push_back(fieldObject{"goal", vector3(20.738, -49.8988, 0), vector3(26.3532, -44.2836, 5.769)}); // Neutral Goal 4
    objects.push_back(fieldObject{"goal", vector3(44.2836, -26.3532, 0), vector3(49.8988, -20.738, 5.769)}); // Neutral Goal 1

    return objects;
}

vector<distanceSensor> fieldRepresentation::distanceSensors() {
    vector<distanceSensor> sensors;
    // Robot frame: +y = forward (intake), +x = right (brain), z = up, inches from tracking center
    sensors.push_back(distanceSensor(0,   vector3(-5.523,  5.131, 5.11),  distanceFront)); // intakeside
    sensors.push_back(distanceSensor(90,  vector3( 7.529,  2.537, 7.235), distanceRight)); // brainside
    sensors.push_back(distanceSensor(180, vector3(-5.523, -5.226, 4.86),  distanceBack));  // clawside
    sensors.push_back(distanceSensor(270, vector3(-7.029,  2.669, 6.122), distanceLeft));  // nonbrainside

    return sensors;
}


fieldRepresentation::distanceSensorDistances fieldRepresentation::simulateCast(vector3 particlePos, double particleHeading) {
    double s = std::sin(deg2rad(particleHeading));
    double c = std::cos(deg2rad(particleHeading));

    double results[4] = {-1, -1, -1, -1}; //0 front, 1 right, 2 back, 3 left, same order as distanceSensors()
    double cosines[4] = {1, 1, 1, 1};
    for (size_t i = 0; i < sensors_.size() && i < 4; i++) {
        const distanceSensor& sensor = sensors_[i];
        vector3 offset = sensor.getCenterOffset();
        double defaultRad = deg2rad(sensor.getDefaultAngle());
        double dx = std::sin(defaultRad);
        double dy = std::cos(defaultRad);

        vector3 sensorPos = particlePos + vector3(offset.x() * c + offset.y() * s, offset.y() * c - offset.x() * s, offset.z());
        vector3 sensorDir(dx * c + dy * s, dy * c - dx * s, 0);
        results[i] = closestHit(sensorCandidates_[i], sensorPos, sensorDir, cosines[i]);
    }

    distanceSensorDistances distances;
    distances.frontDistance = results[0];
    distances.rightDistance = results[1];
    distances.backDistance = results[2];
    distances.leftDistance = results[3];
    distances.frontCos = cosines[0];
    distances.rightCos = cosines[1];
    distances.backCos = cosines[2];
    distances.leftCos = cosines[3];
    distances.updated = true;
    return distances;
}

bool fieldRepresentation::insideObstacle(double x, double y) const {
    for (const fieldObject& object : objects_) {
        if (object.interior) continue;
        if (x >= object.min.x() && x <= object.max.x() && y >= object.min.y() && y <= object.max.y()) return true;
    }
    return false;
}

double fieldRepresentation::closestHit(const vector<fieldObject>& candidates, vector3 sensorPos, vector3 sensorDir, double& hitCos) {
    double closest = maxSensorRange;
    bool hit = false;
    int closestAxis = 0;
    for (const fieldObject& object : candidates) {
        int axis = 0;
        double distance = hitDistance(object, sensorPos, sensorDir, closest, axis);
        if (distance >= 0 && distance < closest) {
            closest = distance;
            closestAxis = axis;
            hit = true;
        }
    }
    hitCos = std::fabs(closestAxis == 0 ? sensorDir.x() : sensorDir.y());
    return hit ? closest : -1;
}

double fieldRepresentation::hitDistance(const fieldObject& object, vector3 sensorPos, vector3 sensorDir, double closestSoFar, int& hitAxis)  { //return -1 if not hit or not closer than closestSoFar
    //https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes//ray-box-intersection.html
    vector3 objectMax = getMax(object);
    vector3 objectMin = getMin(object);
    
    constexpr double parallelEpsilon = 1e-9;
    constexpr double inf = std::numeric_limits<double>::infinity();

    double tmin_x;
    double tmax_x;
    if (std::fabs(sensorDir.x()) < parallelEpsilon) {
        if (sensorPos.x() < objectMin.x() || sensorPos.x() > objectMax.x()) return -1;
        tmin_x = -inf;
        tmax_x = inf;
    } else if (sensorDir.x() > 0) {
        tmin_x = (objectMin.x() - sensorPos.x()) / sensorDir.x(); 
        tmax_x = (objectMax.x() - sensorPos.x()) / sensorDir.x(); 
    } else { 
        tmin_x = (objectMax.x() - sensorPos.x()) / sensorDir.x();
        tmax_x = (objectMin.x() - sensorPos.x()) / sensorDir.x();
    }
   
    if (tmin_x >= closestSoFar) return -1;

    double tmin_y;
    double tmax_y;
    if (std::fabs(sensorDir.y()) < parallelEpsilon) {
        if (sensorPos.y() < objectMin.y() || sensorPos.y() > objectMax.y()) return -1;
        tmin_y = -inf;
        tmax_y = inf;
    } else if (sensorDir.y() > 0) {
        tmin_y = (objectMin.y() - sensorPos.y()) / sensorDir.y(); 
        tmax_y = (objectMax.y() - sensorPos.y()) / sensorDir.y(); 
    } else { 
        tmin_y = (objectMax.y() - sensorPos.y()) / sensorDir.y();
        tmax_y = (objectMin.y() - sensorPos.y()) / sensorDir.y();
    }
    if (tmin_y >= closestSoFar) return -1;

    if (tmin_x > tmax_y || tmin_y > tmax_x) return -1;
    double tmin = (tmin_x > tmin_y) ? tmin_x : tmin_y;
    if (object.interior && tmin < 0) { // sensor inside the box: hit where the ray exits
        hitAxis = (tmax_x < tmax_y) ? 0 : 1;
        return (tmax_x < tmax_y) ? tmax_x : tmax_y;
    }
    if (tmin < 0) return -1;
    hitAxis = (tmin_x > tmin_y) ? 0 : 1;
    return tmin;
}
