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

    // objects.push_back(fieldObject{"alliance_bar", vector3(-13.5945, 70.1062, 10.6686), vector3(13.5945, 72.3031, 14.3549)}); // Blue Alliance Side
    // objects.push_back(fieldObject{"alliance_bar", vector3(70.1062, -13.5945, 10.6686), vector3(72.3031, 13.5945, 14.3549)}); // Blue Alliance Side
    // objects.push_back(fieldObject{"alliance_bar", vector3(-72.3031, -13.5945, 10.6686), vector3(-70.1062, 13.5945, 14.3549)}); // Red Alliance Side
    // objects.push_back(fieldObject{"alliance_bar", vector3(-13.5945, -72.3031, 10.6686), vector3(13.5945, -70.1062, 14.3549)}); // Red Alliance Side

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

fieldRepresentation::distanceSensorDistances fieldRepresentation::simulateCast(distanceSensor sensor, vector3 castDirection, vector3 robotPosition) {

}
double fieldRepresentation::hitDistance(fieldObject)  { //return -1 if not hit

}
