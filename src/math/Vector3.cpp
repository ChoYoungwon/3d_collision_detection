#include "../../include/math/Vector3.h"
#include <cmath>
#include <random>

// 생성자
Vector3::Vector3() : x(0.0f), y(0.0f), z(0.0f) {}

Vector3::Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

Vector3::Vector3(const Vector3& other) : x(other.x), y(other.y), z(other.z) {}

// 연산자 오버로딩
Vector3 Vector3::operator+(const Vector3& other) const {
    return Vector3(x + other.x, y + other.y, z + other.z);
}

Vector3 Vector3::operator-(const Vector3& other) const {
    return Vector3(x - other.x, y - other.y, z - other.z);
}

Vector3 Vector3::operator*(float scalar) const {
    return Vector3(x * scalar, y * scalar, z * scalar);
}

Vector3 Vector3::operator/(float scalar) const {
    float invScalar = 1.0f / scalar;
    return Vector3(x * invScalar, y * invScalar, z * invScalar);
}

Vector3& Vector3::operator+=(const Vector3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

Vector3& Vector3::operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

Vector3& Vector3::operator/=(float scalar) {
    float invScalar = 1.0f / scalar;
    x *= invScalar;
    y *= invScalar;
    z *= invScalar;
    return *this;
}

Vector3 Vector3::operator-() const {
    return Vector3(-x, -y, -z);
}

bool Vector3::operator==(const Vector3& other) const {
    const float epsilon = 1e-6f;
    return std::abs(x - other.x) < epsilon &&
        std::abs(y - other.y) < epsilon &&
        std::abs(z - other.z) < epsilon;
}

bool Vector3::operator!=(const Vector3& other) const {
    return !(*this == other);
}

// 정적 메소드
Vector3 Vector3::zero() {
    return Vector3(0.0f, 0.0f, 0.0f);
}

Vector3 Vector3::one() {
    return Vector3(1.0f, 1.0f, 1.0f);
}

Vector3 Vector3::up() {
    return Vector3(0.0f, 1.0f, 0.0f);
}

Vector3 Vector3::down() {
    return Vector3(0.0f, -1.0f, 0.0f);
}

Vector3 Vector3::left() {
    return Vector3(-1.0f, 0.0f, 0.0f);
}

Vector3 Vector3::right() {
    return Vector3(1.0f, 0.0f, 0.0f);
}

Vector3 Vector3::forward() {
    return Vector3(0.0f, 0.0f, 1.0f);
}

Vector3 Vector3::back() {
    return Vector3(0.0f, 0.0f, -1.0f);
}

Vector3 Vector3::randomUnit() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-1.0f, 1.0f);

    Vector3 v;
    do {
        v.x = dis(gen);
        v.y = dis(gen);
        v.z = dis(gen);
    } while (v.magnitudeSquared() > 1.0f || v.magnitudeSquared() < 0.01f);

    return v.normalized();
}

// 벡터 연산
float Vector3::dot(const Vector3& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vector3 Vector3::cross(const Vector3& other) const {
    return Vector3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

float Vector3::magnitudeSquared() const {
    return x * x + y * y + z * z;
}

float Vector3::magnitude() const {
    return std::sqrt(magnitudeSquared());
}

Vector3 Vector3::normalized() const {
    float mag = magnitude();
    if (mag < 1e-6f) {
        return Vector3::zero();
    }
    return *this / mag;
}

void Vector3::normalize() {
    float mag = magnitude();
    if (mag < 1e-6f) {
        x = y = z = 0.0f;
        return;
    }
    float invMag = 1.0f / mag;
    x *= invMag;
    y *= invMag;
    z *= invMag;
}

float Vector3::distance(const Vector3& other) const {
    return (*this - other).magnitude();
}

float Vector3::distanceSquared(const Vector3& other) const {
    return (*this - other).magnitudeSquared();
}

Vector3 Vector3::reflect(const Vector3& normal) const {
    return *this - normal * (2.0f * dot(normal));
}

Vector3 Vector3::project(const Vector3& onto) const {
    return onto * (dot(onto) / onto.magnitudeSquared());
}

// 문자열 변환
std::string Vector3::toString() const {
    return "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
}

// 비-멤버 연산자 오버로딩
Vector3 operator*(float scalar, const Vector3& v) {
    return v * scalar;
}