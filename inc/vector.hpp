/// @file   vector.hpp
/// @author Evan Brody
/// @brief  Outlines the Vector class

#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <iostream>
#include <cstdint>

namespace AStar {
    // Represents a position in the 2D plane
    class Vector {
        friend std::ostream& operator<<(std::ostream& os, const Vector& rhs);
    public:
        Vector(int x, int y);

        Vector(const Vector& rhs);

        friend bool operator==(const Vector& lhs, const Vector& rhs);
        friend Vector& operator+=(Vector& lhs, const Vector& rhs);
        friend Vector operator-(const Vector& rhs);
        friend double operator*(const Vector& lhs, const Vector& rhs);

        Vector abs() const;

        int16_t getX() const;

        int16_t getY() const;

        int getManhattanDistance(const Vector& other) const;

        double getMagnitude() const;

        // In degrees
        double getAngleBetween(const Vector& other) const;

        double dist(const Vector& other) const;

    private:
        int16_t x, y;
    };

    bool operator!=(const Vector& lhs, const Vector& rhs);
    Vector operator+(Vector lhs, const Vector& rhs);
    Vector& operator-=(Vector& lhs, const Vector& rhs);
    Vector operator-(Vector lhs, const Vector& rhs);
}

#endif