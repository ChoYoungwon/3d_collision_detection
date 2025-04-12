// include/math/Quaternion.h
#ifndef QUATERNION_H
#define QUATERNION_H

#include "Vector3.h"
#include "Matrix3x3.h"
#include <cmath>

class Quaternion {
public:
    float x, y, z, w; // x, y, z: 벡터 부분, w: 스칼라 부분

    // 생성자
    Quaternion() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}
    Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

    // 벡터와 각도로부터 사원수 생성 (단위 축 기준 회전)
    static Quaternion fromAxisAngle(const Vector3& axis, float angleRadians) {
        Vector3 normalizedAxis = axis.normalized();
        float sinHalfAngle = std::sin(angleRadians * 0.5f);
        float cosHalfAngle = std::cos(angleRadians * 0.5f);

        return Quaternion(
            normalizedAxis.x * sinHalfAngle,
            normalizedAxis.y * sinHalfAngle,
            normalizedAxis.z * sinHalfAngle,
            cosHalfAngle
        );
    }

    // 오일러 각도로부터 사원수 생성 (XYZ 순서)
    static Quaternion fromEulerAngles(float pitch, float yaw, float roll) {
        // 각 축에 대한 회전을 사원수로 변환
        float halfPitch = pitch * 0.5f;
        float halfYaw = yaw * 0.5f;
        float halfRoll = roll * 0.5f;

        float cosHalfPitch = std::cos(halfPitch);
        float sinHalfPitch = std::sin(halfPitch);
        float cosHalfYaw = std::cos(halfYaw);
        float sinHalfYaw = std::sin(halfYaw);
        float cosHalfRoll = std::cos(halfRoll);
        float sinHalfRoll = std::sin(halfRoll);

        // XYZ 순서로 회전 (Roll-Pitch-Yaw)
        return Quaternion(
            sinHalfRoll * cosHalfPitch * cosHalfYaw - cosHalfRoll * sinHalfPitch * sinHalfYaw,
            cosHalfRoll * sinHalfPitch * cosHalfYaw + sinHalfRoll * cosHalfPitch * sinHalfYaw,
            cosHalfRoll * cosHalfPitch * sinHalfYaw - sinHalfRoll * sinHalfPitch * cosHalfYaw,
            cosHalfRoll * cosHalfPitch * cosHalfYaw + sinHalfRoll * sinHalfPitch * sinHalfYaw
        );
    }

    // 정규화
    void normalize() {
        float len = std::sqrt(x * x + y * y + z * z + w * w);
        if (len < 1e-6f) {
            x = 0.0f; y = 0.0f; z = 0.0f; w = 1.0f;
            return;
        }
        float invLen = 1.0f / len;
        x *= invLen; y *= invLen; z *= invLen; w *= invLen;
    }

    Quaternion normalized() const {
        Quaternion q = *this;
        q.normalize();
        return q;
    }

    // 켤레(공액) 사원수
    Quaternion conjugate() const {
        return Quaternion(-x, -y, -z, w);
    }

    // 역 사원수
    Quaternion inverse() const {
        // 단위 사원수의 경우 켤레와 역이 같음
        // 일반적인 경우는 켤레를 크기 제곱으로 나눠야 함
        float lenSq = x * x + y * y + z * z + w * w;
        if (lenSq < 1e-6f) {
            return Quaternion(0, 0, 0, 0); // 역이 존재하지 않음
        }

        float invLenSq = 1.0f / lenSq;
        return Quaternion(-x * invLenSq, -y * invLenSq, -z * invLenSq, w * invLenSq);
    }

    // 사원수 곱셈
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w,
            w * q.w - x * q.x - y * q.y - z * q.z
        );
    }

    // 벡터 회전
    Vector3 rotate(const Vector3& v) const {
        // q * v * q^-1 방식으로 벡터 회전
        Quaternion vecQuat(v.x, v.y, v.z, 0.0f);
        Quaternion result = *this * vecQuat * this->conjugate();
        return Vector3(result.x, result.y, result.z);
    }

    // 회전 행렬로 변환
    Matrix3x3 toMatrix3x3() const {
        float xx = x * x;
        float xy = x * y;
        float xz = x * z;
        float xw = x * w;
        float yy = y * y;
        float yz = y * z;
        float yw = y * w;
        float zz = z * z;
        float zw = z * w;

        return Matrix3x3(
            1.0f - 2.0f * (yy + zz), 2.0f * (xy - zw), 2.0f * (xz + yw),
            2.0f * (xy + zw), 1.0f - 2.0f * (xx + zz), 2.0f * (yz - xw),
            2.0f * (xz - yw), 2.0f * (yz + xw), 1.0f - 2.0f * (xx + yy)
        );
    }

    // 구면 선형 보간 (SLERP)
    static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float t) {
        // t는 0과 1 사이의 값
        if (t <= 0.0f) return q1;
        if (t >= 1.0f) return q2;

        Quaternion q2Temp = q2;

        // 내적 계산
        float dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;

        // 최단 경로 보장
        if (dot < 0.0f) {
            q2Temp.x = -q2Temp.x;
            q2Temp.y = -q2Temp.y;
            q2Temp.z = -q2Temp.z;
            q2Temp.w = -q2Temp.w;
            dot = -dot;
        }

        // 두 사원수가 너무 가까우면 선형 보간
        if (dot > 0.9995f) {
            return Quaternion(
                q1.x + t * (q2Temp.x - q1.x),
                q1.y + t * (q2Temp.y - q1.y),
                q1.z + t * (q2Temp.z - q1.z),
                q1.w + t * (q2Temp.w - q1.w)
            ).normalized();
        }

        // 구면 선형 보간
        float angle = std::acos(dot);
        float sinAngle = std::sin(angle);
        float invSinAngle = 1.0f / sinAngle;
        float t1 = std::sin((1.0f - t) * angle) * invSinAngle;
        float t2 = std::sin(t * angle) * invSinAngle;

        return Quaternion(
            q1.x * t1 + q2Temp.x * t2,
            q1.y * t1 + q2Temp.y * t2,
            q1.z * t1 + q2Temp.z * t2,
            q1.w * t1 + q2Temp.w * t2
        );
    }
};

#endif // QUATERNION_H