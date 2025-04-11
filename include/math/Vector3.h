#pragma once
#include <cmath>

namespace Math {
    struct Vector3 {
        float x, y, z;
        
        Vector3() : x(0), y(0), z(0) {}
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
        
        // 벡터 연산자 오버로딩
        Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
        Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
        Vector3 operator*(float s) const { return Vector3(x * s, y * s, z * s); }
        Vector3 operator-() const { return Vector3(-x, -y, -z); }
        
        // 벡터 연산 메서드
        float length() const { return std::sqrt(x*x + y*y + z*z); }
        float lengthSquared() const { return x*x + y*y + z*z; }
        Vector3 normalized() const;
        static float dot(const Vector3& a, const Vector3& b);
        static Vector3 cross(const Vector3& a, const Vector3& b);
    };
}