#ifndef VECTOR3_H
#define VECTOR3_H

#include <string>

class Vector3 {
public:
    float x, y, z;

    // 생성자
    Vector3();
    Vector3(float x, float y, float z);
    Vector3(const Vector3& other);

    // 연산자 오버로딩
    Vector3 operator+(const Vector3& other) const;
    Vector3 operator-(const Vector3& other) const;
    Vector3 operator*(float scalar) const;
    Vector3 operator/(float scalar) const;
    Vector3& operator+=(const Vector3& other);
    Vector3& operator-=(const Vector3& other);
    Vector3& operator*=(float scalar);
    Vector3& operator/=(float scalar);
    Vector3 operator-() const;
    bool operator==(const Vector3& other) const;
    bool operator!=(const Vector3& other) const;

    // 정적 메소드
    static Vector3 zero();
    static Vector3 one();
    static Vector3 up();
    static Vector3 down();
    static Vector3 left();
    static Vector3 right();
    static Vector3 forward();
    static Vector3 back();
    static Vector3 randomUnit();

    // 벡터 연산
    float dot(const Vector3& other) const;
    Vector3 cross(const Vector3& other) const;
    float magnitudeSquared() const;
    float magnitude() const;
    Vector3 normalized() const;
    void normalize();
    float distance(const Vector3& other) const;
    float distanceSquared(const Vector3& other) const;
    Vector3 reflect(const Vector3& normal) const;
    Vector3 project(const Vector3& onto) const;

    // 문자열 변환
    std::string toString() const;
};

// 비-멤버 연산자 오버로딩
Vector3 operator*(float scalar, const Vector3& v);

#endif // VECTOR3_H