#include "../../include/math/Matrix3x3.h"
#include <cmath>
#include <sstream>
#include <iomanip>

// 생성자
Matrix3x3::Matrix3x3() {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

Matrix3x3::Matrix3x3(
    float m00, float m01, float m02,
    float m10, float m11, float m12,
    float m20, float m21, float m22
) {
    m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
    m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
    m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
}

Matrix3x3::Matrix3x3(const Matrix3x3& other) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m[i][j] = other.m[i][j];
        }
    }
}

// 요소 접근
float& Matrix3x3::operator()(int row, int col) {
    return m[row][col];
}

float Matrix3x3::operator()(int row, int col) const {
    return m[row][col];
}

// 행렬 연산
Matrix3x3 Matrix3x3::operator+(const Matrix3x3& other) const {
    Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = m[i][j] + other.m[i][j];
        }
    }
    return result;
}

Matrix3x3 Matrix3x3::operator-(const Matrix3x3& other) const {
    Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = m[i][j] - other.m[i][j];
        }
    }
    return result;
}

Matrix3x3 Matrix3x3::operator*(float scalar) const {
    Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = m[i][j] * scalar;
        }
    }
    return result;
}

Matrix3x3 Matrix3x3::operator*(const Matrix3x3& other) const {
    Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                result.m[i][j] += m[i][k] * other.m[k][j];
            }
        }
    }
    return result;
}

Vector3 Matrix3x3::operator*(const Vector3& v) const {
    return Vector3(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
    );
}

// 전치 행렬 
Matrix3x3 Matrix3x3::transpose() const {
    Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = m[j][i];
        }
    }
    return result;
}

// 행렬식
float Matrix3x3::determinant() const {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

// 역행렬 
Matrix3x3 Matrix3x3::inverse() const {
    float det = determinant();
    if (std::abs(det) < 1e-6f) {
        return identity(); 
    }

    float invDet = 1.0f / det;

    Matrix3x3 result;
    result.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
    result.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
    result.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;
    result.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
    result.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
    result.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet;
    result.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
    result.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet;
    result.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;

    return result;
}

// 단위 행렬 
Matrix3x3 Matrix3x3::identity() {
    return Matrix3x3(
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    );
}

// x축 중심 회전
Matrix3x3 Matrix3x3::rotationX(float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);

    return Matrix3x3(
        1.0f, 0.0f, 0.0f,
        0.0f, c, -s,
        0.0f, s, c
    );
}

// y축 중심 회전
Matrix3x3 Matrix3x3::rotationY(float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);

    return Matrix3x3(
        c, 0.0f, s,
        0.0f, 1.0f, 0.0f,
        -s, 0.0f, c
    );
}

// z축 중심 회전
Matrix3x3 Matrix3x3::rotationZ(float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);

    return Matrix3x3(
        c, -s, 0.0f,
        s, c, 0.0f,
        0.0f, 0.0f, 1.0f
    );
}

// 임의의 축 중심으로 회전
Matrix3x3 Matrix3x3::rotation(const Vector3& axis, float angle) {
    Vector3 a = axis.normalized();
    float c = std::cos(angle);
    float s = std::sin(angle);
    float t = 1.0f - c;

    float x = a.x;
    float y = a.y;
    float z = a.z;

    return Matrix3x3(
        t * x * x + c, t * x * y - s * z, t * x * z + s * y,
        t * x * y + s * z, t * y * y + c, t * y * z - s * x,
        t * x * z - s * y, t * y * z + s * x, t * z * z + c
    );
}

// 각 축에 대한 크기 조정 행렬
Matrix3x3 Matrix3x3::scale(float sx, float sy, float sz) {
    return Matrix3x3(
        sx, 0.0f, 0.0f,
        0.0f, sy, 0.0f,
        0.0f, 0.0f, sz
    );
}

// 각 축에 대한 크기 조정 행렬 (벡터 형태로)
Matrix3x3 Matrix3x3::scale(const Vector3& s) {
    return scale(s.x, s.y, s.z);
}

// 문자열 형식으로 변환환
std::string Matrix3x3::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(4);

    for (int i = 0; i < 3; i++) {
        ss << "| ";
        for (int j = 0; j < 3; j++) {
            ss << std::setw(8) << m[i][j] << " ";
        }
        ss << " |";
        if (i < 2) ss << std::endl;
    }

    return ss.str();
}

Matrix3x3 operator*(float scalar, const Matrix3x3& m) {
    return m * scalar;
}