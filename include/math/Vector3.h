// include/math/Vector3.h
#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>
#include <iostream>

class Vector3 {
public:
    float x, y, z;

    // 생성자
    Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vector3(const Vector3& v) : x(v.x), y(v.y), z(v.z) {}

    // 기본 연산자 오버로딩
    Vector3 operator+(const Vector3& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    Vector3 operator/(float scalar) const {
        if (scalar == 0) {
            std::cerr << "Division by zero!" << std::endl;
            return *this;
        }
        float invScalar = 1.0f / scalar;
        return Vector3(x * invScalar, y * invScalar, z * invScalar);
    }

    Vector3& operator+=(const Vector3& v) {
        x += v.x; y += v.y; z += v.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& v) {
        x -= v.x; y -= v.y; z -= v.z;
        return *this;
    }

    Vector3& operator*=(float scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    Vector3& operator/=(float scalar) {
        if (scalar == 0) {
            std::cerr << "Division by zero!" << std::endl;
            return *this;
        }
        float invScalar = 1.0f / scalar;
        x *= invScalar; y *= invScalar; z *= invScalar;
        return *this;
    }

    bool operator==(const Vector3& v) const {
        return (x == v.x && y == v.y && z == v.z);
    }

    bool operator!=(const Vector3& v) const {
        return !(*this == v);
    }

    // 벡터 길이
    float length() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    float lengthSquared() const {
        return x * x + y * y + z * z;
    }

    // 정규화
    Vector3 normalized() const {
        float len = length();
        if (len < 1e-6f) {
            return Vector3(0, 0, 0);
        }
        return *this / len;
    }

    void normalize() {
        float len = length();
        if (len < 1e-6f) {
            x = y = z = 0;
            return;
        }
        *this /= len;
    }

    // 내적
    static float dot(const Vector3& a, const Vector3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    // 외적
    static Vector3 cross(const Vector3& a, const Vector3& b) {
        return Vector3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }

    // 거리 계산
    static float distance(const Vector3& a, const Vector3& b) {
        return (b - a).length();
    }

    static float distanceSquared(const Vector3& a, const Vector3& b) {
        return (b - a).lengthSquared();
    }

    // 출력을 위한 함수
    friend std::ostream& operator<<(std::ostream& os, const Vector3& v) {
        os << "Vector3(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
};

#endif // VECTOR3_H