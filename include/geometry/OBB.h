#pragma once

#include "geometry/BoundingVolume.h"
#include "math/Vector3.h"
#include "math/Quaternion.h"

class OBB : public BoundingVolume {
public:
    Math::Vector3 center;
    Math::Vector3 halfSize;
    Math::Quaternion orientation;

    OBB();
    OBB(const Math::Vector3& center, const Math::Vector3& halfSize, const Math::Quaternion& orientation);

    Math::Vector3 getAxis(int i) const;
    static OBB fromAABB(const Math::Vector3& min, const Math::Vector3& max);

    // BoundingVolume 메서드 구현 (좌표 포함 여부)
    bool contains(const Math::Vector3& point) const override;
};