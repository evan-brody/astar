/// @file   vector.cpp
/// @author Evan Brody
/// @brief  Implements the Vector class

#include "../inc/vector.hpp"

#include <iostream>
#include <cmath>
#include <numbers> // C++ 20

namespace AStar {
    std::ostream& operator<<(std::ostream& os, const Vector& rhs) {
        return os << '(' << rhs.x << ", " << rhs.y << ')';
    }

    Vector::Vector(int x, int y) : x(x), y(y) {}

    Vector::Vector(const Vector& rhs) : x(rhs.x), y(rhs.y) {}

    bool operator==(const Vector& lhs, const Vector& rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }

    bool operator!=(const Vector& lhs, const Vector& rhs) {
        return !(lhs == rhs);
    }

    // Overloading necessary arithmetic operators
    Vector& operator+=(Vector& lhs, const Vector& rhs) {
        lhs.x += rhs.x; lhs.y += rhs.y;
        return lhs;
    }

    // Negation
    Vector operator-(const Vector& rhs) {
        return Vector(-rhs.x, -rhs.y);
    }

    // Plus and minus
    Vector operator+(Vector lhs, const Vector& rhs) {
        return lhs += rhs;
    }

    Vector& operator-=(Vector& lhs, const Vector& rhs) {
        return lhs += -rhs;
    }

    Vector operator-(Vector lhs, const Vector& rhs) {
        return lhs += (-rhs);
    }

    // Dot product
    double operator*(const Vector& lhs, const Vector& rhs) {
        return lhs.x * rhs.x + lhs.y * rhs.y;
    }

    // Absolute value
    Vector Vector::abs() const {
        return Vector(std::abs(x), std::abs(y));
    }

    short Vector::getX() const { return x; }

    short Vector::getY() const { return y; }

    int Vector::getManhattanDistance(const Vector& other) const {
        return std::abs(this->x - other.x) + std::abs(this->y - other.y);
    }

    double Vector::getMagnitude() const {
        return sqrt(x * x + y * y);
    }

    // In degrees
    double Vector::getAngleBetween(const Vector& other) const {
        double denominator = getMagnitude() * other.getMagnitude();
        if (0 == denominator) { return 0; } // Prevent dividing by 0

        double radians = acos((*this * other) / denominator);

        // Convert to degrees before returning
        return radians * 180.0 / std::numbers::pi;
    }

    double Vector::dist(const Vector& other) const {
        double xDiff = this->x - other.x;
        double yDiff = this->y - other.y;

        return sqrt(xDiff * xDiff + yDiff * yDiff);
    }
}