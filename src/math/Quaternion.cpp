#include "../../include/math/Quaternion.h"
#include <cmath>

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

// 정규화
float Quaternion::magnitudeSquared() const {
    return w * w + x * x + y * y + z * z;
}

float Quaternion::magnitude() const {
    return std::sqrt(magnitudeSquared());
}

Quaternion Quaternion::normalized() const {
    float mag = magnitude();
    if (mag < 1e-6f) {
        return Quaternion::identity();
    }
    return *this / mag;
}

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

// 켤레 및 역원
Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::inverse() const {
    float magSq = magnitudeSquared();
    if (magSq < 1e-6f) {
        return Quaternion::identity();
    }
    return conjugate() / magSq;
}

// 회전
Vector3 Quaternion::rotate(const Vector3& v) const {
    // v' = q * v * q^-1
    // 최적화된 구현
    Vector3 u(x, y, z);
    Vector3 uv = u.cross(v);
    Vector3 uuv = u.cross(uv);
    return v + ((uv * w) + uuv) * 2.0f;
}

Quaternion Quaternion::rotateBy(const Quaternion& rotation) const {
    return rotation * (*this) * rotation.inverse();
}

// 변환
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

// 정적 메소드
Quaternion Quaternion::identity() {
    return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
}

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

Quaternion Quaternion::fromEulerAngles(float x, float y, float z) {
    // ZYX 순서로 회전 (z->y->x)
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

// 구면 선형 보간 (SLERP)
Quaternion Quaternion::slerp(const Quaternion& q1, const Quaternion& q2, float t) {
    // q1과 q2가 정규화되어 있다고 가정
    Quaternion q2Temp = q2;

    // 내적 계산
    float dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

    // 최단 경로를 보장하기 위해 필요한 경우 방향 반전
    if (dot < 0.0f) {
        q2Temp = q2Temp * -1.0f;
        dot = -dot;
    }

    // 두 쿼터니언이 거의 같은 경우 선형 보간 사용
    if (dot > 0.9995f) {
        Quaternion result = q1 * (1.0f - t) + q2Temp * t;
        return result.normalized();
    }

    // 구면 선형 보간
    float theta = std::acos(dot);
    float sinTheta = std::sin(theta);
    float ratioA = std::sin((1.0f - t) * theta) / sinTheta;
    float ratioB = std::sin(t * theta) / sinTheta;

    return (q1 * ratioA + q2Temp * ratioB).normalized();
}

// 룩앤 로테이션
Quaternion Quaternion::lookRotation(const Vector3& forward, const Vector3& up) {
    Vector3 normalizedForward = forward.normalized();
    Vector3 normalizedUp = up.normalized();

    // 정방향 벡터가 0이면 항등 쿼터니언 반환
    if (normalizedForward.magnitudeSquared() < 1e-6f) {
        return Quaternion::identity();
    }

    // 직교 기저 구축
    Vector3 right = normalizedUp.cross(normalizedForward).normalized();
    Vector3 orthogonalUp = normalizedForward.cross(right);

    // 회전 행렬 생성
    Matrix3x3 m(
        right.x, orthogonalUp.x, normalizedForward.x,
        right.y, orthogonalUp.y, normalizedForward.y,
        right.z, orthogonalUp.z, normalizedForward.z
    );

    return fromRotationMatrix(m);
}

// 유틸리티 함수
float Quaternion::dot(const Quaternion& other) const {
    return w * other.w + x * other.x + y * other.y + z * other.z;
}

float Quaternion::angle() const {
    return 2.0f * std::acos(w);
}

Vector3 Quaternion::axis() const {
    const float s = std::sqrt(1.0f - w * w);
    if (s < 1e-6f) {
        return Vector3(1.0f, 0.0f, 0.0f);  // 임의의 축 반환
    }
    return Vector3(x / s, y / s, z / s);
}

Vector3 Quaternion::toEulerAngles() const {
    // 쿼터니언에서 오일러 각도(XYZ 순서) 추출
    Vector3 angles;

    // X 회전 (Roll)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    angles.x = std::atan2(sinr_cosp, cosr_cosp);

    // Y 회전 (Pitch)
    float sinp = 2.0f * (w * y - z * x);
    if (std::abs(sinp) >= 1.0f) {
        angles.y = std::copysign(M_PI / 2.0f, sinp);  // 90도로 제한 (짐벌락 지점)
    }
    else {
        angles.y = std::asin(sinp);
    }

    // Z 회전 (Yaw)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    angles.z = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}