#include "mcl.h"
#include "fieldRepresentation.h"

fieldRepresentation::distanceSensorDistances MCL::sensorData_;
vector<particle> MCL::particles_;
lemlib::Pose MCL::prevPose(0, 0, 0);

fieldRepresentation& MCL::getField() {
    static fieldRepresentation field; // built on first call, after robot.cpp's distance sensors exist
    return field;
}

void MCL::init(int numParticles) {
    particles_.clear();
    particles_.reserve(numParticles);
    // particle heading = imu rotation + headingOffset, so this offset makes particles start at lemlib's heading
    double startOffset = chassis.getPose().theta - imu.get_rotation();
    // TODO: spread particles across the field (or around a known start pose), headingOffset centred on startOffset

    
    pros::Task mcl_task([=]() {
        prevPose = chassis.getPose();
        while (true) {
            sensorData_ = readSensors();
            double imuHeading = imu.get_rotation(); // continuous, unlike get_heading which wraps at 360

            lemlib::Pose pose = chassis.getPose();
            lemlib::Pose delta = pose - prevPose;
            double th = deg2rad(prevPose.theta);
            double forward = delta.x * std::sin(th) + delta.y * std::cos(th);
            double right = delta.x * std::cos(th) - delta.y * std::sin(th);
            update(forward, right, imuHeading);

            lemlib::Pose correction = weigh(getField()) - pose;
            chassis.setPose(chassis.getPose() + correction);
            prevPose = pose + correction;
            //todo: consider adding some way to smooth correction application to avoid jerkiness and reject overly large corrections

            if (effectiveParticleCount() < particles_.size() / 2.0) resample();
            pros::delay(20);
        }
    });
}

void MCL::update(double forward, double right, double imuHeading) {
    for (auto& p : particles_) {
        // TODO: rotate (forward, right) by p.getHeading(imuHeading) into the field frame, move p by it plus noise
    }
}

lemlib::Pose MCL::weigh(fieldRepresentation& field) {
    double imuHeading = imu.get_rotation();
    lemlib::Pose estimatedPosition(0, 0, 0);
    for (auto& p : particles_) {
        fieldRepresentation::distanceSensorDistances simDistances = field.simulateCast(p.getPosition(), p.getHeading(imuHeading));
        // TODO: compare simDistances to sensorData_ and set p's weight
    }
    return estimatedPosition;
}

double MCL::effectiveParticleCount() {
    double sumW = 0, sumW2 = 0;
    for (const auto& p : particles_) {
        sumW += p.getWeight();
        sumW2 += p.getWeight() * p.getWeight();
    }
    if (sumW2 <= 0) return 0;
    return sumW * sumW / sumW2;
}

fieldRepresentation::distanceSensorDistances MCL::readSensors() {
    int32_t front = distanceFront.get();
    int32_t right = distanceRight.get();
    int32_t back = distanceBack.get();
    int32_t left = distanceLeft.get();

    fieldRepresentation::distanceSensorDistances data;
    data.frontDistance = mmToInches(front);
    data.rightDistance = mmToInches(right);
    data.backDistance = mmToInches(back);
    data.leftDistance = mmToInches(left);
    data.updated = front != PROS_ERR && right != PROS_ERR && back != PROS_ERR && left != PROS_ERR;
    return data;
}

void MCL::resample() {
    // TODO
}
