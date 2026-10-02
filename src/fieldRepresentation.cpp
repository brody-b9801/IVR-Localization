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

fieldRepresentation::distanceSensorDistances fieldRepresentation::simulateCast(distanceSensor sensor, vector3 castDirection) {
    distanceSensorDistances distances;
    return distances;
}
double fieldRepresentation::hitDistance(fieldObject object, distanceSensor sensor)  { //return -1 if not hit
    //https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes//ray-box-intersection.html
    vector3 sensorPos = sensor.getPosition();
    vector3 sensorDir = sensor.getDirection();
    
    vector3 objectMax = getMax(object);
    if (sensorPos.z() > objectMax.z()) return -1;
    vector3 objectMin = getMin(object);
    
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
