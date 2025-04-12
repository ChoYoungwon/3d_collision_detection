#include "math/Quaternion.h"
#include "math/Matrix3x3.h"
#include <cmath>

namespace Math {

    Quaternion Quaternion::fromAxisAngle(const Vector3& axis, float angle) {
        Vector3 normAxis = axis.normalized();
        float halfAngle = angle * 0.5f;
        float sinHalf = std::sin(halfAngle);
        return Quaternion(
            std::cos(halfAngle),
            normAxis.x * sinHalf,
            normAxis.y * sinHalf,
            normAxis.z * sinHalf
        );
    }

    Matrix3x3 Quaternion::toRotationMatrix() const {
        Matrix3x3 mat;

        float xx = x * x;
        float yy = y * y;
        float zz = z * z;
        float xy = x * y;
        float xz = x * z;
        float yz = y * z;
        float wx = w * x;
        float wy = w * y;
        float wz = w * z;

        mat.m[0][0] = 1.0f - 2.0f * (yy + zz);
        mat.m[0][1] = 2.0f * (xy - wz);
        mat.m[0][2] = 2.0f * (xz + wy);

        mat.m[1][0] = 2.0f * (xy + wz);
        mat.m[1][1] = 1.0f - 2.0f * (xx + zz);
        mat.m[1][2] = 2.0f * (yz - wx);

        mat.m[2][0] = 2.0f * (xz - wy);
        mat.m[2][1] = 2.0f * (yz + wx);
        mat.m[2][2] = 1.0f - 2.0f * (xx + yy);

        return mat;
    }

    Quaternion Quaternion::operator*(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    Vector3 Quaternion::rotate(const Vector3& v) const {
        Quaternion p(0, v.x, v.y, v.z);
        Quaternion result = (*this) * p * this->normalized().conjugate();
        return Vector3(result.x, result.y, result.z);
    }

    Quaternion Quaternion::normalized() const {
        float len = std::sqrt(w * w + x * x + y * y + z * z);
        if (len == 0) return Quaternion();
        return Quaternion(w / len, x / len, y / len, z / len);
    }

    Quaternion Quaternion::conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }
}