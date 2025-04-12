// include/math/Matrix4x4.h
#pragma once

#include "Vector3.h"
#include "Vector4.h"
#include "Quaternion.h"
#include "Matrix3x3.h"

namespace Math {

class Matrix4x4 {
public:
    // ��� ������ (�� �켱)
    float m[4][4];

    // ������
    Matrix4x4();
    Matrix4x4(const Matrix4x4& other);
    Matrix4x4(
        float m00, float m01, float m02, float m03,
        float m10, float m11, float m12, float m13,
        float m20, float m21, float m22, float m23,
        float m30, float m31, float m32, float m33
    );

    // ���� ��� ��ȯ
    static Matrix4x4 identity();

    // ��ȯ ��� ���� (��ġ, ȸ��, ũ��)
    static Matrix4x4 createTransformation(const Vector3& position, const Quaternion& rotation, const Vector3& scale);

    // �̵� ��� ����
    static Matrix4x4 createTranslation(const Vector3& position);

    // ȸ�� ��� ����
    static Matrix4x4 createRotation(const Quaternion& rotation);

    // ũ�� ���� ��� ����
    static Matrix4x4 createScale(const Vector3& scale);

    // �� ��� ����
    static Matrix4x4 createView(const Vector3& eye, const Vector3& target, const Vector3& up);

    // ���� ��� ����
    static Matrix4x4 createPerspective(float fov, float aspectRatio, float nearPlane, float farPlane);
    static Matrix4x4 createOrthographic(float left, float right, float bottom, float top, float nearPlane, float farPlane);

    // ������ �����ε�
    Matrix4x4 operator+(const Matrix4x4& other) const;
    Matrix4x4 operator-(const Matrix4x4& other) const;
    Matrix4x4 operator*(const Matrix4x4& other) const;
    Matrix4x4 operator*(float scalar) const;
    Vector4 operator*(const Vector4& vector) const;
    Matrix4x4& operator=(const Matrix4x4& other);
    bool operator==(const Matrix4x4& other) const;
    bool operator!=(const Matrix4x4& other) const;

    // ��� ����
    Matrix4x4 transpose() const;
    Matrix4x4 inverse() const;
    float determinant() const;

    // 3x3 ��� ���� (ȸ��/ũ�� �κ�)
    Matrix3x3 toMatrix3x3() const;

    // ��ġ ����
    Vector3 getTranslation() const;

    // ȸ�� ����
    Quaternion getRotation() const;

    // ũ�� ����
    Vector3 getScale() const;

    // ��ȯ ����
    Vector3 transformPoint(const Vector3& point) const;
    Vector3 transformVector(const Vector3& vector) const;

    // ���ڿ� ��ȯ
    std::string toString() const;
};

} // namespace Math