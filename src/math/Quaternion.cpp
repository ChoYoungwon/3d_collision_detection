#include "../../include/math/Quaternion.h"
#include <cmath>
#include <sstream>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 생성자
Quaternion::Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
Quaternion::Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
Quaternion::Quaternion(const Quaternion& other) : w(other.w), x(other.x), y(other.y), z(other.z) {}

// 연산자 오버로딩
Quaternion Quaternion::operator+(const Quaternion& other) const {
    return Quaternion(w + other.w, x + other.x, y + other.y, z + other.z);
}

Quaternion Quaternion::operator-(const Quaternion& other) const {
    return Quaternion(w - other.w, x - other.x, y - other.y, z - other.z);
}

// 쿼터니언 곱셈 : p * q(q 회전을 수행한 후 p 회전을 수행하는 것)
Quaternion Quaternion::operator*(const Quaternion& other) const {
    return Quaternion(
        w * other.w - x * other.x - y * other.y - z * other.z,
        w * other.x + x * other.w + y * other.z - z * other.y,
        w * other.y - x * other.z + y * other.w + z * other.x,
        w * other.z + x * other.y - y * other.x + z * other.w
    );
}

Quaternion Quaternion::operator*(float scalar) const {
    return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
}

Quaternion Quaternion::operator/(float scalar) const {
    float invScalar = 1.0f / scalar;
    return Quaternion(w * invScalar, x * invScalar, y * invScalar, z * invScalar);
}

bool Quaternion::operator==(const Quaternion& other) const {
    const float epsilon = 1e-6f;
    return std::abs(w - other.w) < epsilon &&
        std::abs(x - other.x) < epsilon &&
        std::abs(y - other.y) < epsilon &&
        std::abs(z - other.z) < epsilon;
}

bool Quaternion::operator!=(const Quaternion& other) const {
    return !(*this == other);
}

// Non-member operator
Quaternion operator*(float scalar, const Quaternion& q) {
    return q * scalar;
}

// 쿼터니언 크기의 제곱
float Quaternion::magnitudeSquared() const {
    return w * w + x * x + y * y + z * z;
}

// 쿼터니언의 크기
float Quaternion::magnitude() const {
    return std::sqrt(magnitudeSquared());
}

// 정규화된 쿼터니언 사본을 반환(크기가 1)
Quaternion Quaternion::normalized() const {
    float mag = magnitude();
    if (mag < 1e-6f) {
        return Quaternion::identity();
    }
    return *this / mag;
}

// 현재 쿼터니언을 정규화
void Quaternion::normalize() {
    float mag = magnitude();
    if (mag < 1e-6f) {
        w = 1.0f;
        x = y = z = 0.0f;
        return;
    }
    float invMag = 1.0f / mag;
    w *= invMag;
    x *= invMag;
    y *= invMag;
    z *= invMag;
}

// 쿼터니언의 켤레를 반환
Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

// 쿼터니언의 역을 반환
Quaternion Quaternion::inverse() const {
    float magSq = magnitudeSquared();
    if (magSq < 1e-6f) {
        return Quaternion::identity();
    }
    return conjugate() / magSq;
}

// 벡터를 쿼터니언으로 회전
Vector3 Quaternion::rotate(const Vector3& v) const {
    // v' = q * v * q^-1
    Vector3 u(x, y, z);
    Vector3 uv = u.cross(v);
    Vector3 uuv = u.cross(uv);
    return v + ((uv * w) + uuv) * 2.0f;
}

// 현재 쿼터니언을 다른 쿼터니언에 의해 회전
Quaternion Quaternion::rotateBy(const Quaternion& rotation) const {
    return rotation * (*this) * rotation.inverse();
}

// 회전 행렬로 변환
Matrix3x3 Quaternion::toRotationMatrix() const {
    Quaternion q = normalized();
    float xx = q.x * q.x;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float xw = q.x * q.w;
    float yy = q.y * q.y;
    float yz = q.y * q.z;
    float yw = q.y * q.w;
    float zz = q.z * q.z;
    float zw = q.z * q.w;

    return Matrix3x3(
        1.0f - 2.0f * (yy + zz), 2.0f * (xy - zw), 2.0f * (xz + yw),
        2.0f * (xy + zw), 1.0f - 2.0f * (xx + zz), 2.0f * (yz - xw),
        2.0f * (xz - yw), 2.0f * (yz + xw), 1.0f - 2.0f * (xx + yy)
    );
}

// 오일러 각도로 변환
Vector3 Quaternion::toEulerAngles() const {
    // XYZ 순서
    Vector3 angles;

    // X rotation (Roll)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    angles.x = std::atan2(sinr_cosp, cosr_cosp);

    // Y rotation (Pitch)
    float sinp = 2.0f * (w * y - z * x);
    if (std::abs(sinp) >= 1.0f) {
        angles.y = std::copysign(M_PI / 2.0f, sinp);  // 90 degrees if at poles
    }
    else {
        angles.y = std::asin(sinp);
    }

    // Z rotation (Yaw)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    angles.z = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

// 유틸리티 함수
// 쿼터니언의 내적 
float Quaternion::dot(const Quaternion& other) const {
    return w * other.w + x * other.x + y * other.y + z * other.z;
}

// 회전 각도를 반환(세타 = 2 x arccos(w))
float Quaternion::angle() const {
    return 2.0f * std::acos(w);
}

// 회전 축을 반환
Vector3 Quaternion::axis() const {
    const float s = std::sqrt(1.0f - w * w);
    if (s < 1e-6f) {
        return Vector3(1.0f, 0.0f, 0.0f); 
    }
    return Vector3(x / s, y / s, z / s);
}

// 문자열 표현
std::string Quaternion::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << "[w:" << w << ", x:" << x << ", y:" << y << ", z:" << z << "]";
    return oss.str();
}

// 정적 메서드
// 단위 쿼터니언을 반환
Quaternion Quaternion::identity() {
    return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
}

// 회전 축과 각도로부터 쿼터니언을 생성
Quaternion Quaternion::fromAxisAngle(const Vector3& axis, float angle) {
    Vector3 normalizedAxis = axis.normalized();
    float halfAngle = angle * 0.5f;
    float s = std::sin(halfAngle);

    return Quaternion(
        std::cos(halfAngle),
        normalizedAxis.x * s,
        normalizedAxis.y * s,
        normalizedAxis.z * s
    );
}

// 오일러 각도로부터 쿼터니언을 생성
Quaternion Quaternion::fromEulerAngles(float x, float y, float z) {
    // ZYX order rotation (z->y->x)
    float cx = std::cos(x * 0.5f);
    float cy = std::cos(y * 0.5f);
    float cz = std::cos(z * 0.5f);
    float sx = std::sin(x * 0.5f);
    float sy = std::sin(y * 0.5f);
    float sz = std::sin(z * 0.5f);

    return Quaternion(
        cx * cy * cz + sx * sy * sz,
        sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * sz,
        cx * cy * sz - sx * sy * cz
    );
}

// 오일러 각도로부터 쿼터니언을 생성
Quaternion Quaternion::fromEulerAngles(const Vector3& euler) {
    // Call the other overload with the euler components
    return fromEulerAngles(euler.x, euler.y, euler.z);
}

// 회전 행렬로부터 쿼터니언을 생성
Quaternion Quaternion::fromRotationMatrix(const Matrix3x3& m) {
    float trace = m(0, 0) + m(1, 1) + m(2, 2);
    Quaternion q;

    if (trace > 0.0f) {
        float s = 0.5f / std::sqrt(trace + 1.0f);
        q.w = 0.25f / s;
        q.x = (m(2, 1) - m(1, 2)) * s;
        q.y = (m(0, 2) - m(2, 0)) * s;
        q.z = (m(1, 0) - m(0, 1)) * s;
    }
    else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
        float s = 2.0f * std::sqrt(1.0f + m(0, 0) - m(1, 1) - m(2, 2));
        q.w = (m(2, 1) - m(1, 2)) / s;
        q.x = 0.25f * s;
        q.y = (m(0, 1) + m(1, 0)) / s;
        q.z = (m(0, 2) + m(2, 0)) / s;
    }
    else if (m(1, 1) > m(2, 2)) {
        float s = 2.0f * std::sqrt(1.0f + m(1, 1) - m(0, 0) - m(2, 2));
        q.w = (m(0, 2) - m(2, 0)) / s;
        q.x = (m(0, 1) + m(1, 0)) / s;
        q.y = 0.25f * s;
        q.z = (m(1, 2) + m(2, 1)) / s;
    }
    else {
        float s = 2.0f * std::sqrt(1.0f + m(2, 2) - m(0, 0) - m(1, 1));
        q.w = (m(1, 0) - m(0, 1)) / s;
        q.x = (m(0, 2) + m(2, 0)) / s;
        q.y = (m(1, 2) + m(2, 1)) / s;
        q.z = 0.25f * s;
    }

    return q.normalized();
}

// 두 쿼터니언사이를 구면 선형 보간(0 <= t <= 1)
Quaternion Quaternion::slerp(const Quaternion& q1, const Quaternion& q2, float t) {
    // Assuming q1 and q2 are normalized
    Quaternion q2Temp = q2;

    // Compute dot product
    float dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

    // If necessary, flip sign to get shortest path
    if (dot < 0.0f) {
        q2Temp = q2Temp * -1.0f;
        dot = -dot;
    }

    // If quaternions are very close, use linear interpolation
    if (dot > 0.9995f) {
        Quaternion result = q1 * (1.0f - t) + q2Temp * t;
        return result.normalized();
    }

    // Perform spherical interpolation
    float theta = std::acos(dot);
    float sinTheta = std::sin(theta);
    float ratioA = std::sin((1.0f - t) * theta) / sinTheta;
    float ratioB = std::sin(t * theta) / sinTheta;

    return (q1 * ratioA + q2Temp * ratioB).normalized();
}

// 전방 및 상단 벡터로부터 방향 쿼터니언을 생성
Quaternion Quaternion::lookRotation(const Vector3& forward, const Vector3& up) {
    Vector3 normalizedForward = forward.normalized();
    Vector3 normalizedUp = up.normalized();

    // 전방 벡터가 0이면 identity를 반환환
    if (normalizedForward.magnitudeSquared() < 1e-6f) {
        return Quaternion::identity();
    }

    // Compute orthogonal basis
    Vector3 right = normalizedUp.cross(normalizedForward).normalized();
    Vector3 orthogonalUp = normalizedForward.cross(right);

    // Create rotation matrix
    Matrix3x3 m(
        right.x, orthogonalUp.x, normalizedForward.x,
        right.y, orthogonalUp.y, normalizedForward.y,
        right.z, orthogonalUp.z, normalizedForward.z
    );

    return fromRotationMatrix(m);
}

// Static version of toEulerAngles
Vector3 Quaternion::toEulerAngles(const Quaternion& q) {
    return q.toEulerAngles();
}