// include/math/Matrix3x3.h
#ifndef MATRIX3X3_H
#define MATRIX3X3_H

#include "Vector3.h"
#include <array>
#include <cmath>

class Matrix3x3 {
private:
    std::array<float, 9> m; // 행 우선 순서로 저장 (row-major)

public:
    // 기본 생성자 (항등 행렬)
    Matrix3x3() {
        identity();
    }

    // 개별 요소로 초기화
    Matrix3x3(float m00, float m01, float m02,
        float m10, float m11, float m12,
        float m20, float m21, float m22) {
        m[0] = m00; m[1] = m01; m[2] = m02;
        m[3] = m10; m[4] = m11; m[5] = m12;
        m[6] = m20; m[7] = m21; m[8] = m22;
    }

    // 배열로 초기화
    Matrix3x3(const std::array<float, 9>& data) : m(data) {}

    // 복사 생성자
    Matrix3x3(const Matrix3x3& other) : m(other.m) {}

    // 항등 행렬 설정
    void identity() {
        m[0] = 1.0f; m[1] = 0.0f; m[2] = 0.0f;
        m[3] = 0.0f; m[4] = 1.0f; m[5] = 0.0f;
        m[6] = 0.0f; m[7] = 0.0f; m[8] = 1.0f;
    }

    // 행렬 요소 접근
    float& operator()(int row, int col) {
        return m[row * 3 + col];
    }

    const float& operator()(int row, int col) const {
        return m[row * 3 + col];
    }

    // 행렬 곱셈
    Matrix3x3 operator*(const Matrix3x3& other) const {
        Matrix3x3 result;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result(i, j) = 0.0f;
                for (int k = 0; k < 3; k++) {
                    result(i, j) += (*this)(i, k) * other(k, j);
                }
            }
        }
        return result;
    }

    // 벡터와 행렬 곱셈
    Vector3 operator*(const Vector3& v) const {
        return Vector3(
            m[0] * v.x + m[1] * v.y + m[2] * v.z,
            m[3] * v.x + m[4] * v.y + m[5] * v.z,
            m[6] * v.x + m[7] * v.y + m[8] * v.z
        );
    }

    // 전치 행렬
    Matrix3x3 transpose() const {
        return Matrix3x3(
            m[0], m[3], m[6],
            m[1], m[4], m[7],
            m[2], m[5], m[8]
        );
    }

    // 행렬식 계산
    float determinant() const {
        return m[0] * (m[4] * m[8] - m[5] * m[7]) -
            m[1] * (m[3] * m[8] - m[5] * m[6]) +
            m[2] * (m[3] * m[7] - m[4] * m[6]);
    }

    // 역행렬 계산
    Matrix3x3 inverse() const {
        float det = determinant();
        if (std::abs(det) < 1e-6f) {
            // 역행렬이 존재하지 않음
            return Matrix3x3();
        }

        float invDet = 1.0f / det;

        Matrix3x3 result;
        result(0, 0) = (m[4] * m[8] - m[5] * m[7]) * invDet;
        result(0, 1) = (m[2] * m[7] - m[1] * m[8]) * invDet;
        result(0, 2) = (m[1] * m[5] - m[2] * m[4]) * invDet;
        result(1, 0) = (m[5] * m[6] - m[3] * m[8]) * invDet;
        result(1, 1) = (m[0] * m[8] - m[2] * m[6]) * invDet;
        result(1, 2) = (m[2] * m[3] - m[0] * m[5]) * invDet;
        result(2, 0) = (m[3] * m[7] - m[4] * m[6]) * invDet;
        result(2, 1) = (m[1] * m[6] - m[0] * m[7]) * invDet;
        result(2, 2) = (m[0] * m[4] - m[1] * m[3]) * invDet;

        return result;
    }

    // 회전 행렬 생성 (오일러 각도)
    static Matrix3x3 createRotationX(float radians) {
        float c = std::cos(radians);
        float s = std::sin(radians);
        return Matrix3x3(
            1.0f, 0.0f, 0.0f,
            0.0f, c, -s,
            0.0f, s, c
        );
    }

    static Matrix3x3 createRotationY(float radians) {
        float c = std::cos(radians);
        float s = std::sin(radians);
        return Matrix3x3(
            c, 0.0f, s,
            0.0f, 1.0f, 0.0f,
            -s, 0.0f, c
        );
    }

    static Matrix3x3 createRotationZ(float radians) {
        float c = std::cos(radians);
        float s = std::sin(radians);
        return Matrix3x3(
            c, -s, 0.0f,
            s, c, 0.0f,
            0.0f, 0.0f, 1.0f
        );
    }

    // 스케일 행렬 생성
    static Matrix3x3 createScale(float sx, float sy, float sz) {
        return Matrix3x3(
            sx, 0.0f, 0.0f,
            0.0f, sy, 0.0f,
            0.0f, 0.0f, sz
        );
    }

    static Matrix3x3 createScale(const Vector3& scale) {
        return createScale(scale.x, scale.y, scale.z);
    }
};

#endif // MATRIX3X3_H