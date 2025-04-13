#include "geometry/OBB.h"
#include <cmath>

// getAxis: 회전 행렬의 i번째 열을 반환 (i = 0, 1, 2)
Vector3 OBB::getAxis(int i) const {
    return Vector3(orientation(0, i), orientation(1, i), orientation(2, i));
}

// fromAABB: 최소 좌표(min)와 최대 좌표(max)로부터 AABB를 만들고, 이를 이용해 OBB를 생성
OBB OBB::fromAABB(const Vector3& min, const Vector3& max) {
    Vector3 center = (min + max) * 0.5f;
    Vector3 halfExtents = (max - min) * 0.5f;
    Matrix3x3 identity; // 기본 생성자가 단위 행렬을 생성한다고 가정
    return OBB(center, halfExtents, identity);
}