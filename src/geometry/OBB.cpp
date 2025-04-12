#include "geometry/OBB.h"
#include "math/Matrix3x3.h"
#include <cmath>

OBB::OBB()
    : center(), halfSize(), orientation() {}

OBB::OBB(const Math::Vector3& center, const Math::Vector3& halfSize, const Math::Quaternion& orientation)
    : center(center), halfSize(halfSize), orientation(orientation) {}

Math::Vector3 OBB::getAxis(int i) const {
    // 임시 변수 없이 직접 사용
    return Math::Vector3(
        orientation.toRotationMatrix().m[0][i],
        orientation.toRotationMatrix().m[1][i],
        orientation.toRotationMatrix().m[2][i]
    );
}

OBB OBB::fromAABB(const Math::Vector3& min, const Math::Vector3& max) {
    Math::Vector3 center = (min + max) * 0.5f;
    Math::Vector3 halfSize = (max - min) * 0.5f;
    Math::Quaternion orientation; // 단위 쿼터니언 (회전 없음)
    return OBB(center, halfSize, orientation);
}

bool OBB::contains(const Math::Vector3& point) const {
    Math::Vector3 d = point - center;

    // getAxis()를 사용하여 축을 가져옴
    for (int i = 0; i < 3; ++i) {
        Math::Vector3 axis = getAxis(i);
        float dist = Math::Vector3::dot(d, axis);
        float halfExtent = (i == 0) ? halfSize.x : (i == 1) ? halfSize.y : halfSize.z;

        if (std::abs(dist) > halfExtent) {
            return false;
        }
    }
    return true;
}