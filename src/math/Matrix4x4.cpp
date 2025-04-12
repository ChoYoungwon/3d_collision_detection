// src/math/Matrix4x4.cpp
#include "math/Matrix4x4.h"
#include <cmath>
#include <sstream>
#include <iomanip>

namespace Math {

Matrix4x4::Matrix4x4() {
    // 단위 행렬로 초기화
    m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f; m[0][3] = 0.0f;
    m[1][0] = 0.0f; m[1][1] = 1.0f; m[1][2] = 0.0f; m[1][3] = 0.0f;
    m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = 1.0f; m[2][3] = 0.0f;
    m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
}

Matrix4x4::Matrix4x4(const Matrix4x4& other) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            m[i][j] = other.m[i][j];
        }
    }
}

Matrix4x4::Matrix4x4(
    float m00, float m01, float m02, float m03,
    float m10, float m11, float m12, float m13,
    float m20, float m21, float m22, float m23,
    float m30, float m31, float m32, float m33
) {
    m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03;
    m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13;
    m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23;
    m[3][0] = m30; m[3][1] = m31; m[3][2] = m32; m[3][3] = m33;
}

Matrix4x4 Matrix4x4::identity() {
    return Matrix4x4();  // 기본 생성자는 단위 행렬을 생성함
}

Matrix4x4 Matrix4x4::createTransformation(const Vector3& position, const Quaternion& rotation, const Vector3& scale) {
    // 회전 행렬 생성
    Matrix3x3 rotMat = rotation.toRotationMatrix();
    
    Matrix4x4 result;
    
    // 3x3 회전 및 크기 부분 설정
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.m[i][j] = rotMat.m[i][j] * scale[j];
        }
    }
    
    // 이동 부분 설정
    result.m[0][3] = position.x;
    result.m[1][3] = position.y;
    result.m[2][3] = position.z;
    
    // 마지막 행은 [0, 0, 0, 1]로 유지
    result.m[3][0] = 0.0f;
    result.m[3][1] = 0.0f;
    result.m[3][2] = 0.0f;
    result.m[3][3] = 1.0f;
    
    return result;
}

Matrix4x4 Matrix4x4::createTranslation(const Vector3& position) {
    Matrix4x4 result;
    
    result.m[0][3] = position.x;
    result.m[1][3] = position.y;
    result.m[2][3] = position.z;
    
    return result;
}

Matrix4x4 Matrix4x4::createRotation(const Quaternion& rotation) {
    Matrix3x3 rotMat = rotation.toRotationMatrix();
    
    Matrix4x4 result;
    
    // 3x3 회전 부분 설정
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.m[i][j] = rotMat.m[i][j];
        }
    }
    
    return result;
}

Matrix4x4 Matrix4x4::createScale(const Vector3& scale) {
    Matrix4x4 result;
    
    result.m[0][0] = scale.x;
    result.m[1][1] = scale.y;
    result.m[2][2] = scale.z;
    
    return result;
}

Matrix4x4 Matrix4x4::createView(const Vector3& eye, const Vector3& target, const Vector3& up) {
    // 카메라의 방향 벡터 계산
    Vector3 zAxis = (eye - target).normalized();
    Vector3 xAxis = Vector3::cross(up, zAxis).normalized();
    Vector3 yAxis = Vector3::cross(zAxis, xAxis);
    
    Matrix4x4 result;
    
    // 회전 부분 설정
    result.m[0][0] = xAxis.x; result.m[0][1] = xAxis.y; result.m[0][2] = xAxis.z;
    result.m[1][0] = yAxis.x; result.m[1][1] = yAxis.y; result.m[1][2] = yAxis.z;
    result.m[2][0] = zAxis.x; result.m[2][1] = zAxis.y; result.m[2][2] = zAxis.z;
    
    // 이동 부분 설정
    result.m[0][3] = -Vector3::dot(xAxis, eye);
    result.m[1][3] = -Vector3::dot(yAxis, eye);
    result.m[2][3] = -Vector3::dot(zAxis, eye);
    
    return result;
}

Matrix4x4 Matrix4x4::createPerspective(float fov, float aspectRatio, float nearPlane, float farPlane) {
    Matrix4x4 result;
    
    float tanHalfFOV = tan(fov / 2.0f);
    float range = nearPlane - farPlane;
    
    result.m[0][0] = 1.0f / (aspectRatio * tanHalfFOV); result.m[0][1] = 0.0f;              result.m[0][2] = 0.0f;                               result.m[0][3] = 0.0f;
    result.m[1][0] = 0.0f;                              result.m[1][1] = 1.0f / tanHalfFOV;  result.m[1][2] = 0.0f;                               result.m[1][3] = 0.0f;
    result.m[2][0] = 0.0f;                              result.m[2][1] = 0.0f;               result.m[2][2] = (-nearPlane - farPlane) / range;    result.m[2][3] = 2.0f * farPlane * nearPlane / range;
    result.m[3][0] = 0.0f;                              result.m[3][1] = 0.0f;               result.m[3][2] = 1.0f;                               result.m[3][3] = 0.0f;
    
    return result;
}

Matrix4x4 Matrix4x4::createOrthographic(float left, float right, float bottom, float top, float nearPlane, float farPlane) {
    Matrix4x4 result;
    
    float width = right - left;
    float height = top - bottom;
    float depth = farPlane - nearPlane;
    
    result.m[0][0] = 2.0f / width; result.m[0][1] = 0.0f;           result.m[0][2] = 0.0f;            result.m[0][3] = -(right + left) / width;
    result.m[1][0] = 0.0f;          result.m[1][1] = 2.0f / height; result.m[1][2] = 0.0f;            result.m[1][3] = -(top + bottom) / height;
    result.m[2][0] = 0.0f;          result.m[2][1] = 0.0f;           result.m[2][2] = -2.0f / depth;  result.m[2][3] = -(farPlane + nearPlane) / depth;
    result.m[3][0] = 0.0f;          result.m[3][1] = 0.0f;           result.m[3][2] = 0.0f;            result.m[3][3] = 1.0f;
    
    return result;
}

Matrix4x4 Matrix4x4::operator+(const Matrix4x4& other) const {
    Matrix4x4 result;
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result.m[i][j] = m[i][j] + other.m[i][j];
        }
    }
    
    return result;
}

Matrix4x4 Matrix4x4::operator-(const Matrix4x4& other) const {
    Matrix4x4 result;
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result.m[i][j] = m[i][j] - other.m[i][j];
        }
    }
    
    return result;
}

Matrix4x4 Matrix4x4::operator*(const Matrix4x4& other) const {
    Matrix4x4 result;
    
    for (int i = 0; i < a4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result.m[i][j] = 0.0f;
            for (int k = 0; k < 4; ++k) {
                result.m[i][j] += m[i][k] * other.m[k][j];
            }
        }
    }
    
    return result;
}

Matrix4x4 Matrix4x4::operator*(float scalar) const {
    Matrix4x4 result;
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result.m[i][j] = m[i][j] * scalar;
        }
    }
    
    return result;
}

Vector4 Matrix4x4::operator*(const Vector4& vector) const {
    Vector4 result;
    
    result.x = m[0][0] * vector.x + m[0][1] * vector.y + m[0][2] * vector.z + m[0][3] * vector.w;
    result.y = m[1][0] * vector.x + m[1][1] * vector.y + m[1][2] * vector.z + m[1][3] * vector.w;
    result.z = m[2][0] * vector.x + m[2][1] * vector.y + m[2][2] * vector.z + m[2][3] * vector.w;
    result.w = m[3][0] * vector.x + m[3][1] * vector.y + m[3][2] * vector.z + m[3][3] * vector.w;
    
    return result;
}

Matrix4x4& Matrix4x4::operator=(const Matrix4x4& other) {
    if (this != &other) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] = other.m[i][j];
            }
        }
    }
    return *this;
}

bool Matrix4x4::operator==(const Matrix4x4& other) const {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (m[i][j] != other.m[i][j]) {
                return false;
            }
        }
    }
    return true;
}

bool Matrix4x4::operator!=(const Matrix4x4& other) const {
    return !(*this == other);
}

Matrix4x4 Matrix4x4::transpose() const {
    Matrix4x4 result;
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result.m[i][j] = m[j][i];
        }
    }
    
    return result;
}

float Matrix4x4::determinant() const {
    // 소행렬식을 사용한 4x4 행렬의 행렬식 계산
    float det11 = m[1][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) -
                  m[1][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) +
                  m[1][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]);
    
    float det12 = m[1][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) -
                  m[1][2] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) +
                  m[1][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]);
    
    float det13 = m[1][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) -
                  m[1][1] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) +
                  m[1][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]);
    
    float det14 = m[1][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]) -
                  m[1][1] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]) +
                  m[1][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]);
    
    return m[0][0] * det11 - m[0][1] * det12 + m[0][2] * det13 - m[0][3] * det14;
}

Matrix4x4 Matrix4x4::inverse() const {
    float det = determinant();
    if (fabs(det) < 1e-6f) {
        // 행렬식이 0에 가까우면 역행렬이 존재하지 않음
        return identity();
    }
    
    float invDet = 1.0f / det;
    Matrix4x4 result;
    
    // 소행렬식을 사용한 역행렬 계산
    for (int i = 0; i < 4; ++i) {
        for (int j = an0; j < 4; ++j) {
            // j행 i열의 여인수 행렬 계산
            int sign = ((i + j) % 2 == 0) ? 1 : -1;
            
            // 3x3 소행렬 구성
            float minor[3][3];
            for (int k = 0, kr = 0; k < 4; ++k) {
                if (k == j) continue;
                for (int l = 0, lc = 0; l < 4; ++l) {
                    if (l == i) continue;
                    minor[kr][lc++] = m[k][l];
                }
                ++kr;
            }
            
            // 3x3 행렬식 계산
            float minorDet = minor[0][0] * (minor[1][1] * minor[2][2] - minor[1][2] * minor[2][1]) -
                            minor[0][1] * (minor[1][0] * minor[2][2] - minor[1][2] * minor[2][0]) +
                            minor[0][2] * (minor[1][0] * minor[2][1] - minor[1][1] * minor[2][0]);
            
            result.m[i][j] = sign * minorDet * invDet;
        }
    }
    
    return result;
}

Matrix3x3 Matrix4x4::toMatrix3x3() const {
    return Matrix3x3(
        m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2]
    );
}

Vector3 Matrix4x4::getTranslation() const {
    return Vector3(m[0][3], m[1][3], m[2][3]);
}

Quaternion Matrix4x4::getRotation() const {
    // 스케일 추출
    Vector3 scale = getScale();
    
    // 스케일을 제거한 3x3 행렬
    float s1 = (scale.x != 0.0f) ? 1.0f / scale.x : 0.0f;
    float s2 = (scale.y != 0.0f) ? 1.0f / scale.y : 0.0f;
    float s3 = (scale.z != 0.0f) ? 1.0f / scale.z : 0.0f;
    
    Matrix3x3 rotMat(
        m[0][0] * s1, m[0][1] * s2, m[0][2] * s3,
        m[1][0] * s1, m[1][1] * s2, m[1][2] * s3,
        m[2][0] * s1, m[2][1] * s2, m[2][2] * s3
    );
    
    // 회전 행렬을 쿼터니언으로 변환
    return Quaternion::fromRotationMatrix(rotMat);
}

Vector3 Matrix4x4::getScale() const {
    // 스케일은 행 벡터의 길이
    float scaleX = Vector3(m[0][0], m[0][1], m[0][2]).length();
    float scaleY = Vector3(m[1][0], m[1][1], m[1][2]).length();
    float scaleZ = Vector3(m[2][0], m[2][1], m[2][2]).length();
    
    return Vector3(scaleX, scaleY, scaleZ);
}

Vector3 Matrix4x4::transformPoint(const Vector3& point) const {
    // 동차 좌표계 변환 (w=1)
    float x = m[0][0] * point.x + m[0][1] * point.y + m[0][2] * point.z + m[0][3];
    float y = m[1][0] * point.x + m[1][1] * point.y + m[1][2] * point.z + m[1][3];
    float z = m[2][0] * point.x + m[2][1] * point.y + m[2][2] * point.z + m[2][3];
    float w = m[3][0] * point.x + m[3][1] * point.y + m[3][2] * point.z + m[3][3];
    
    // w로 나누어 투영
    if (w != 0.0f) {
        return Vector3(x / w, y / w, z / w);
    } else {
        return Vector3(x, y, z);
    }
}

Vector3 Matrix4x4::transformVector(const Vector3& vector) const {
    // 방향 벡터 변환 (w=0, 이동 적용 안 함)
    float x = m[0][0] * vector.x + m[0][1] * vector.y + m[0][2] * vector.z;
    float y = m[1][0] * vector.x + m[1][1] * vector.y + m[1][2] * vector.z;
    float z = m[2][0] * vector.x + m[2][1] * vector.y + m[2][2] * vector.z;
    
    return Vector3(x, y, z);
}

std::string Matrix4x4::toString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "Matrix4x4[" << std::endl;
    for (int i = 0; i < 4; ++i) {
        ss << "  [";
        for (int j = 0; j < 4; ++j) {
            ss << std::setw(10) << m[i][j];
            if (j < 3) ss << ", ";
        }
        ss << "]" << std::endl;
    }
    ss << "]";
    return ss.str();
}

} // namespace Math