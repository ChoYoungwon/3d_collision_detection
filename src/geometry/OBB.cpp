#include "geometry/OBB.h"
#include "math/Matrix3x3.h"
#include <cmath>

OBB::OBB()
    : center(), halfSize(), orientation() {}

OBB::OBB(const Math::Vector3& center, const Math::Vector3& halfSize, const Math::Quaternion& orientation)
    : center(center), halfSize(halfSize), orientation(orientation) {}

Math::Vector3 OBB::getAxis(int i) const {
    // Quaternion�� ȸ�� ��ķ� ��ȯ�Ͽ� �� �� ��ȯ
    Math::Matrix3x3 rot = orientation.toRotationMatrix();
    return Math::Vector3(rot.m[0][i], rot.m[1][i], rot.m[2][i]);
}

OBB OBB::fromAABB(const Math::Vector3& min, const Math::Vector3& max) {
    Math::Vector3 center = (min + max) * 0.5f;
    Math::Vector3 halfSize = (max - min) * 0.5f;
    Math::Quaternion orientation; // ���� ���ʹϾ� (ȸ�� ����)
    return OBB(center, halfSize, orientation);
}

bool OBB::contains(const Math::Vector3& point) const {
    // ���� ��ǥ��� ��ȯ
    Math::Matrix3x3 rot = orientation.toRotationMatrix();  // const ���
    Math::Vector3 d = point - center;

    for (int i = 0; i < 3; ++i) {
        Math::Vector3 axis = getAxis(i);
        float dist = Math::Vector3::dot(d, axis);
        float halfExtent = 0.0f;
        if (i == 0)
            halfExtent = halfSize.x;
        else if (i == 1)
            halfExtent = halfSize.y;
        else if (i == 2)
            halfExtent = halfSize.z;
        if (std::abs(dist) > halfExtent) {
            return false;
        }
    }
    return true;
}