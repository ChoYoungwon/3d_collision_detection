#pragma once

#include <cmath>

namespace collision_detection {
    class Vector3 {
    public:
        float x, y, z;

        // 생성자
        Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
        Vector3(const Vector3& other) : x(other.x), y(other.y), z(other.z) {}

        // 연산자 오버로딩
        Vector3 operator+(const Vector3& other) const {
            return Vector3(x + other.x, y + other.y, z + other.z);
        }

        Vector3 operator-(const Vector3& other) const {
            return Vector3(x - other.x, y - other.y, z - other.z);
        }

        Vector3 operator*(float scalar) const {
            return Vector3(x * scalar, y * scalar, z * scalar);
        }

        Vector3 operator/(float scalar) const {
            return Vector3(x / scalar, y / scalar, z / scalar);
        }

        // 벡터 길이 관련 메소드
        float Length() const {
            return std::sqrt(x * x + y * y + z * z);
        }

        float LengthSq() const {
            return x * x + y * y + z * z;
        }

        Vector3 Normalize() const {
            float length = Length();
            if (length > 0)
                return Vector3(x / length, y / length, z / length);
            return *this;
        }

        // 내적, 외적
        float Dot(const Vector3& other) const {
            return x * other.x + y * other.y + z * other.z;
        }

        Vector3 Cross(const Vector3& other) const {
            return Vector3(
                y * other.z - z * other.y, 
                z * other.x - x * other.z, 
                x * other.y - y * other.x
            );
        }
    };
}