#pragma once

#include "geometry/BoundingVolume.h"
#include "math/Vector3.h"

class AABB : public BoundingVolume {
public:
    Math::Vector3 min;
    Math::Vector3 max;
    
    AABB();
    AABB(const Math::Vector3& min, const Math::Vector3& max);
    
    // AABB 메서드
    bool contains(const Math::Vector3& point) const override;
    bool intersects(const AABB& other) const;
    void expand(const Math::Vector3& point);
    void expand(const AABB& other);
    Math::Vector3 getCenter() const;
    Math::Vector3 getExtents() const;
};

