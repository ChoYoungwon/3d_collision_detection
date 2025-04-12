// include/math/Matrix4x4.h
#pragma once

#include "Vector3.h"
#include "Vector4.h"
#include "Quaternion.h"
#include "Matrix3x3.h"

namespace Math {

class Matrix4x4 {
public:
    // 행렬 데이터 (행 우선)
    float m[4][4];

    // 생성자
    Matrix4x4();
    Matrix4x4(const Matrix4x4& other);
    Matrix4x4(
        float m00, float m01, float m02, float m03,
        float m10, float m11, float m12, float m13,
        float m20, float m21, float m22, float m23,
        float m30, float m31, float m32, float m33
    );

    // 단위 행렬 반환
    static Matrix4x4 identity();

    // 변환 행렬 생성 (위치, 회전, 크기)
    static Matrix4x4 createTransformation(const Vector3& position, const Quaternion& rotation, const Vector3& scale);

    // 이동 행렬 생성
    static Matrix4x4 createTranslation(const Vector3& position);

    // 회전 행렬 생성
    static Matrix4x4 createRotation(const Quaternion& rotation);

    // 크기 조절 행렬 생성
    static Matrix4x4 createScale(const Vector3& scale);

    // 뷰 행렬 생성
    static Matrix4x4 createView(const Vector3& eye, const Vector3& target, const Vector3& up);

    // 투영 행렬 생성
    static Matrix4x4 createPerspective(float fov, float aspectRatio, float nearPlane, float farPlane);
    static Matrix4x4 createOrthographic(float left, float right, float bottom, float top, float nearPlane, float farPlane);

    // 연산자 오버로딩
    Matrix4x4 operator+(const Matrix4x4& other) const;
    Matrix4x4 operator-(const Matrix4x4& other) const;
    Matrix4x4 operator*(const Matrix4x4& other) const;
    Matrix4x4 operator*(float scalar) const;
    Vector4 operator*(const Vector4& vector) const;
    Matrix4x4& operator=(const Matrix4x4& other);
    bool operator==(const Matrix4x4& other) const;
    bool operator!=(const Matrix4x4& other) const;

    // 행렬 연산
    Matrix4x4 transpose() const;
    Matrix4x4 inverse() const;
    float determinant() const;

    // 3x3 행렬 추출 (회전/크기 부분)
    Matrix3x3 toMatrix3x3() const;

    // 위치 추출
    Vector3 getTranslation() const;

    // 회전 추출
    Quaternion getRotation() const;

    // 크기 추출
    Vector3 getScale() const;

    // 변환 적용
    Vector3 transformPoint(const Vector3& point) const;
    Vector3 transformVector(const Vector3& vector) const;

    // 문자열 변환
    std::string toString() const;
};

} // namespace Math