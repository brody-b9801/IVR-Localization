#include "fieldRepresentation.h"

fieldRepresentation::fieldRepresentation(vector<distanceSensor> sensors) : fieldRepresentation(sensors, fieldObjects()) {}
fieldRepresentation::fieldRepresentation(vector<distanceSensor> sensors, vector<fieldObject> objects) : objects_(objects), sensors_(sensors) {}

// Axis-aligned boxes in inches: origin at field centre on top of the foam tiles,
// +X towards the blue alliance station, +Z up. min/max are opposite corners.
vector<fieldRepresentation::fieldObject> fieldRepresentation::fieldObjects() {
    vector<fieldObject> objects;

    objects.push_back(fieldObject{"wall", vector3(70.1968, -70.1968, -0.622), vector3(72.2338, 70.1968, 11.5368)}); // +X wall
    objects.push_back(fieldObject{"wall", vector3(-72.2338, -70.1968, -0.622), vector3(-70.1968, 70.1968, 11.5368)}); // -X wall
    objects.push_back(fieldObject{"wall", vector3(-72.2338, 70.1968, -0.622), vector3(72.2338, 72.2338, 11.5368)}); // +Y wall
    objects.push_back(fieldObject{"wall", vector3(-72.2338, -72.2338, -0.622), vector3(72.2338, -70.1968, 11.5368)}); // -Y wall

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
    //0 front, 1 right, 2 back, 3 left
    sensors.push_back(distanceSensor(0, vector3(0, 0, 0), distanceFront));
    sensors.push_back(distanceSensor(90, vector3(0, 0, 0), distanceRight));
    sensors.push_back(distanceSensor(180, vector3(0, 0, 0), distanceBack));
    sensors.push_back(distanceSensor(270, vector3(0, 0, 0), distanceLeft));

    return sensors;
}


fieldRepresentation::distanceSensorDistances fieldRepresentation::simulateCast(vector3 particlePos, double particleHeading) {
    double s = std::sin(deg2rad(particleHeading));
    double c = std::cos(deg2rad(particleHeading));

    double results[4] = {-1, -1, -1, -1}; //0 front, 1 right, 2 back, 3 left, same order as distanceSensors()
    for (size_t i = 0; i < sensors_.size() && i < 4; i++) {
        const distanceSensor& sensor = sensors_[i];
        vector3 offset = sensor.getCenterOffset();
        double defaultRad = deg2rad(sensor.getDefaultAngle());
        double dx = std::sin(defaultRad);
        double dy = std::cos(defaultRad);

        vector3 sensorPos = particlePos + vector3(offset.x() * c + offset.y() * s, offset.y() * c - offset.x() * s, offset.z());
        vector3 sensorDir(dx * c + dy * s, dy * c - dx * s, 0);
        results[i] = closestHit(sensorPos, sensorDir);
    }

    distanceSensorDistances distances;
    distances.frontDistance = results[0];
    distances.rightDistance = results[1];
    distances.backDistance = results[2];
    distances.leftDistance = results[3];
    distances.updated = true;
    return distances;
}

double fieldRepresentation::closestHit(vector3 sensorPos, vector3 sensorDir) {
    double closest = maxSensorRange;
    bool hit = false;
    for (const fieldObject& object : objects_) {
        double distance = hitDistance(object, sensorPos, sensorDir);
        if (distance >= 0 && distance < closest) {
            closest = distance;
            hit = true;
        }
    }
    return hit ? closest : -1;
}

double fieldRepresentation::hitDistance(const fieldObject& object, vector3 sensorPos, vector3 sensorDir)  { //return -1 if not hit
    //https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes//ray-box-intersection.html
    vector3 objectMax = getMax(object);
    vector3 objectMin = getMin(object);
    if (sensorPos.z() > objectMax.z() || sensorPos.z() < objectMin.z()) return -1;
    
    double tmin_x;
    double tmax_x;
    if (sensorDir.x() >= 0) { 
        tmin_x = (objectMin.x() - sensorPos.x()) / sensorDir.x(); 
        tmax_x = (objectMax.x() - sensorPos.x()) / sensorDir.x(); 
    } else { 
        tmin_x = (objectMax.x() - sensorPos.x()) / sensorDir.x();
        tmax_x = (objectMin.x() - sensorPos.x()) / sensorDir.x(); 
    }

    double tmin_y;
    double tmax_y;
    if (sensorDir.y() >= 0) { 
        tmin_y = (objectMin.y() - sensorPos.y()) / sensorDir.y(); 
        tmax_y = (objectMax.y() - sensorPos.y()) / sensorDir.y(); 
    } else { 
        tmin_y = (objectMax.y() - sensorPos.y()) / sensorDir.y();
        tmax_y = (objectMin.y() - sensorPos.y()) / sensorDir.y(); 
    }

    if (tmin_x > tmax_y || tmin_y > tmax_x) return -1;
    double tmin = (tmin_x > tmin_y) ? tmin_x : tmin_y;
    if (tmin < 0) return -1;
    return tmin;
}
