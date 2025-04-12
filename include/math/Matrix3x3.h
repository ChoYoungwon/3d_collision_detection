#ifndef MATRIX3X3_H
#define MATRIX3X3_H

#include "Vector3.h"
#include <array>
#include <string>

class Matrix3x3 {
private:
    std::array<std::array<float, 3>, 3> m;

public:
    // 생성자
    Matrix3x3();
    Matrix3x3(
        float m00, float m01, float m02,
        float m10, float m11, float m12,
        float m20, float m21, float m22
    );
    Matrix3x3(const Matrix3x3& other);

    // 요소 접근
    float& operator()(int row, int col);
    float operator()(int row, int col) const;

    // 행렬 연산
    Matrix3x3 operator+(const Matrix3x3& other) const;
    Matrix3x3 operator-(const Matrix3x3& other) const;
    Matrix3x3 operator*(float scalar) const;
    Matrix3x3 operator*(const Matrix3x3& other) const;
    Vector3 operator*(const Vector3& v) const;

    // 전치 행렬
    Matrix3x3 transpose() const;

    // 행렬식
    float determinant() const;

    // 역행렬
    Matrix3x3 inverse() const;

    // 정적 메소드
    static Matrix3x3 identity();
    static Matrix3x3 rotationX(float angle);
    static Matrix3x3 rotationY(float angle);
    static Matrix3x3 rotationZ(float angle);
    static Matrix3x3 rotation(const Vector3& axis, float angle);
    static Matrix3x3 scale(float sx, float sy, float sz);
    static Matrix3x3 scale(const Vector3& s);

    // 문자열 변환
    std::string toString() const;
};

// 비-멤버 연산자 오버로딩
Matrix3x3 operator*(float scalar, const Matrix3x3& m);

#endif // MATRIX3X3_H