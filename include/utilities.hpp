#pragma once
#include <cmath>
#include <numbers>
#include "api.h"

namespace utilities {
    constexpr double deg2rad(double degrees) {
        return degrees * std::numbers::pi / 180.0;
    }

    constexpr double rad2deg(double radians) {
        return radians * 180.0 / std::numbers::pi;
    }

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

            double dotProduct(const vector3& other) const {
                return x_ * other.x_ + y_ * other.y_ + z_ * other.z_;
            }

            vector3 crossProduct(const vector3& other) const {
                return vector3(
                    y_ * other.z_ - z_ * other.y_,
                    z_ * other.x_ - x_ * other.z_,
                    x_ * other.y_ - y_ * other.x_
                );
            }

            vector3 operator+(const vector3& other) const {
                return vector3(x_ + other.x_, y_ + other.y_, z_ + other.z_);
            }

            vector3 operator-(const vector3& other) const {
                return vector3(x_ - other.x_, y_ - other.y_, z_ - other.z_);
            }


            vector3& operator+=(const vector3& other) {
                x_ += other.x_;
                y_ += other.y_;
                z_ += other.z_;
                return *this;
            }

            vector3& operator-=(const vector3& other) {
                x_ -= other.x_;
                y_ -= other.y_;
                z_ -= other.z_;
                return *this;
            }

            vector3 operator*(double scalar) const {
                return vector3(x_ * scalar, y_ * scalar, z_ * scalar);
            }

            vector3& operator*=(double scalar) {
                x_ *= scalar;
                y_ *= scalar;
                z_ *= scalar;
                return *this;
            }

            double length() const {
                return std::sqrt(dotProduct(*this));
            }

            // unit vector in the same direction; a zero vector stays zero
            vector3 normalized() const {
                double len = length();
                return len == 0 ? *this : *this * (1.0 / len);
            }

            // this vector rotated angle degrees about axis (counterclockwise looking down the axis)
            vector3 rotated(const vector3& axis, double angle) const {
                vector3 unitAxis = axis.normalized();
                double rad = deg2rad(angle);
                double cosAngle = std::cos(rad);
                double sinAngle = std::sin(rad);
                return *this * cosAngle + unitAxis.crossProduct(*this) * sinAngle + unitAxis * (unitAxis.dotProduct(*this) * (1 - cosAngle));
            }

            
    };

    inline vector3 operator*(double scalar, const vector3& v) {
        return v * scalar;
    }
    
    class distanceSensor {
        private:
            double defaultAngle_;
            vector3 centerOffset_;
            double angle_;
            vector3 position;
            vector3 direction;            
        public: 
            distanceSensor(double angle, vector3 centerOffset) : defaultAngle_(angle), centerOffset_(centerOffset), angle_(angle) {}

            void update(double robotAngle, vector3 robotPosition) {
                angle_ = defaultAngle_ + robotAngle;
                // headings are compass-style like LemLib (0 = +Y, clockwise positive),
                // which is a negative rotation about +Z
                position = robotPosition + centerOffset_.rotated(vector3(0, 0, 1), -robotAngle);
                direction = vector3(std::sin(deg2rad(angle_)), std::cos(deg2rad(angle_)), 0);
            }
            double getAngle() const { return angle_; }
            vector3 getPosition() const { return position; }
            vector3 getDirection() const { return direction; }
    };
}
