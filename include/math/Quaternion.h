#ifndef QUATERNION_H
#define QUATERNION_H

#include "Vector3.h"
#include "Matrix3x3.h"
#include <string>


// 쿼터니언 : 3D 회전을 표현
class Quaternion {
public:
    // w는 스칼라 부분
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

    // 정규화 관련 함수
    float magnitudeSquared() const;
    float magnitude() const;
    Quaternion normalized() const;
    void normalize();

    // 켤레와 역 
    Quaternion conjugate() const;
    Quaternion inverse() const;

    // 회전 관련 함수
    Vector3 rotate(const Vector3& v) const;
    Quaternion rotateBy(const Quaternion& rotation) const;

    // 변환 함수
    Matrix3x3 toRotationMatrix() const;
    Vector3 toEulerAngles() const;

    // 유틸리티 함수
    float dot(const Quaternion& other) const;
    float angle() const;
    Vector3 axis() const;
    std::string toString() const;

    // 정적 메서드
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