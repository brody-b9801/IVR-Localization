#pragma once
#include "api.h"

namespace utilities {
    class vector3 {
        private:
            double x_;
            double y_;
            double z_;

        public:
            vector3(double x = 0, double y = 0, double z = 0) : x_(x), y_(y), z_(z) {}

            double x() const { return x_; }
            double y() const { return y_; }
            double z() const { return z_; }

            vector3 operator+(const vector3& other) const {
                return vector3(x_ + other.x_, y_ + other.y_, z_ + other.z_);
            }

            vector3 operator-(const vector3& other) const {
                return vector3(x_ - other.x_, y_ - other.y_, z_ - other.z_);
            }

    };
}
