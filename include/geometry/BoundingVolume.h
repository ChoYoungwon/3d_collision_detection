#pragma once

#include "math/Vector3.h"

class BoundingVolume {
public:
    virtual ~BoundingVolume() = default;
    virtual bool contains(const Math::Vector3& point) const = 0;
};