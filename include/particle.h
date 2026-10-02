#pragma once
#include "utilities.hpp"

using namespace utilities;


class particle {
    private:
        double x_; // inches
        double y_;
        double headingOffset_; // degrees
        double weight_;

    public:
        particle(double x = 0, double y = 0, double headingOffset = 0, double weight = 1) : x_(x), y_(y), headingOffset_(headingOffset), weight_(weight) {}

        double getX() const { return x_; }
        double getY() const { return y_; }
        double getHeadingOffset() const { return headingOffset_; }
        double getWeight() const { return weight_; }

        vector3 getPosition() const { return vector3(x_, y_, 0); }
        double getHeading(double imuHeading) const { return imuHeading + headingOffset_; }

        void setPosition(double x, double y) { x_ = x; y_ = y; }
        void setHeadingOffset(double headingOffset) { headingOffset_ = headingOffset; }
        void setWeight(double weight) { weight_ = weight; }
};
