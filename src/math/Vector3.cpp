#include "math/Vector3.h"

namespace Math {

Vector3::Vector3() : x(0), y(0), z(0) {}

Vector3::Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

Vector3 Vector3::operator+(const Vector3& v) const {
    return Vector3(x + v.x, y + v.y, z + v.z);
}

Vector3 Vector3::operator-(const Vector3& v) const {
    return Vector3(x - v.x, y - v.y, z - v.z);
}

Vector3 Vector3::operator*(float s) const {
    return Vector3(x * s, y * s, z * s);
}

Vector3 Vector3::operator-() const {
    return Vector3(-x, -y, -z);
}

float Vector3::length() const {
    return std::sqrt(x*x + y*y + z*z);
}

float Vector3::lengthSquared() const {
    return x*x + y*y + z*z;
}

Vector3 Vector3::normalized() const {
    float len = length();
    if (len < 1e-6f) return Vector3(0, 0, 0);
    return Vector3(x / len, y / len, z / len);
}

float Vector3::dot(const Vector3& a, const Vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector3 Vector3::cross(const Vector3& a, const Vector3& b) {
    return Vector3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

} // namespace Math