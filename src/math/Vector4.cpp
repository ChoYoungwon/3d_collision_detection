// src/math/Vector4.cpp
#include "math/Vector4.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace Math {

    // 상수 정의
    const Vector4 Vector4::ZERO(0.0f, 0.0f, 0.0f, 0.0f);
    const Vector4 Vector4::ONE(1.0f, 1.0f, 1.0f, 1.0f);
    const Vector4 Vector4::UNIT_X(1.0f, 0.0f, 0.0f, 0.0f);
    const Vector4 Vector4::UNIT_Y(0.0f, 1.0f, 0.0f, 0.0f);
    const Vector4 Vector4::UNIT_Z(0.0f, 0.0f, 1.0f, 0.0f);
    const Vector4 Vector4::UNIT_W(0.0f, 0.0f, 0.0f, 1.0f);

    Vector4::Vector4() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {
    }

    Vector4::Vector4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {
    }

    Vector4::Vector4(const Vector3& vec3, float w) : x(vec3.x), y(vec3.y), z(vec3.z), w(w) {
    }

    Vector4::Vector4(const Vector4& other) : x(other.x), y(other.y), z(other.z), w(other.w) {
    }

    Vector4 Vector4::operator+(const Vector4& other) const {
        return Vector4(x + other.x, y + other.y, z + other.z, w + other.w);
    }

    Vector4 Vector4::operator-(const Vector4& other) const {
        return Vector4(x - other.x, y - other.y, z - other.z, w - other.w);
    }

    Vector4 Vector4::operator*(float scalar) const {
        return Vector4(x * scalar, y * scalar, z * scalar, w * scalar);
    }

    Vector4 Vector4::operator/(float scalar) const {
        if (scalar == 0.0f) {
            throw std::invalid_argument("Division by zero");
        }
        float invScalar = 1.0f / scalar;
        return Vector4(x * invScalar, y * invScalar,