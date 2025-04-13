#ifndef QUATERNION_H
#define QUATERNION_H

#include "Vector3.h"
#include "Matrix3x3.h"
#include <string>

class Quaternion {
public:
    float w, x, y, z;

    // Constructors
    Quaternion();
    Quaternion(float w, float x, float y, float z);
    Quaternion(const Quaternion& other);

    // Operator overloading
    Quaternion operator+(const Quaternion& other) const;
    Quaternion operator-(const Quaternion& other) const;
    Quaternion operator*(const Quaternion& other) const;
    Quaternion operator*(float scalar) const;
    Quaternion operator/(float scalar) const;
    bool operator==(const Quaternion& other) const;
    bool operator!=(const Quaternion& other) const;

    // Normalization
    float magnitudeSquared() const;
    float magnitude() const;
    Quaternion normalized() const;
    void normalize();

    // Conjugate and inverse
    Quaternion conjugate() const;
    Quaternion inverse() const;

    // Rotation
    Vector3 rotate(const Vector3& v) const;
    Quaternion rotateBy(const Quaternion& rotation) const;

    // Transformations
    Matrix3x3 toRotationMatrix() const;
    Vector3 toEulerAngles() const;

    // Utility functions
    float dot(const Quaternion& other) const;
    float angle() const;
    Vector3 axis() const;
    std::string toString() const;

    // Static methods
    static Quaternion identity();
    static Quaternion fromAxisAngle(const Vector3& axis, float angle);
    static Quaternion fromEulerAngles(float x, float y, float z);
    static Quaternion fromEulerAngles(const Vector3& euler);
    static Quaternion fromRotationMatrix(const Matrix3x3& m);
    static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float t);
    static Quaternion lookRotation(const Vector3& forward, const Vector3& up);
    static Vector3 toEulerAngles(const Quaternion& q);
};

// Non-member operator overloading
Quaternion operator*(float scalar, const Quaternion& q);

#endif // QUATERNION_H