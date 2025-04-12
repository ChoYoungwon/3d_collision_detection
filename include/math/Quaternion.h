#ifndef QUATERNION_H
#define QUATERNION_H

#include "Vector3.h"
#include "Matrix3x3.h"
#include <string>

class Quaternion {
public:
    float w, x, y, z;

    // 생성자
    Quaternion();
    Quaternion(float w, float x, float y, float z);
    Quaternion(const Quaternion& other);

    // 연산자 오버로딩
    Quaternion operator+(const Quaternion& other) const;
    Quaternion operator-(const Quaternion& other) const;
    Quaternion operator*(const Quaternion& other) const;
    Quaternion operator*(float scalar) const;
    Quaternion operator/(float scalar) const;
    bool operator==(const Quaternion& other) const;
    bool operator!=(const Quaternion& other) const;

    // 정규화
    float magnitudeSquared() const;
    float magnitude() const;
    Quaternion normalized() const;
    void normalize();

    // 켤레 및 역원
    Quaternion conjugate() const;
    Quaternion inverse() const;

    // 회전
    Vector3 rotate(const Vector3& v) const;
    Quaternion rotateBy(const Quaternion& rotation) const;

    // 변환
    Matrix3x3 toRotationMatrix() const;

    // 정적 메소드
    static Quaternion identity();
    static Quaternion fromAxisAngle(const Vector3& axis, float angle);
    static Quaternion fromEulerAngles(float x, float y, float z);
    static Quaternion fromEulerAngles(const Vector3& euler);
    static Quaternion fromRotationMatrix(const Matrix3x3& m);
    static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float t);
    static Vector3 toEulerAngles(const Quaternion& q);

    // 문자열 변환
    std::string toString() const;
};

// 비-멤버 연산자 오버로딩
Quaternion operator*(float scalar, const Quaternion& q);

#endif // QUATERNION_H