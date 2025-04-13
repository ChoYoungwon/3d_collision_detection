#pragma once

#include "Vector3.h"

namespace collision_detection {
    class AABB {
    public:
        Vector3 min;
        Vector3 max;

        AABB() : min(0, 0, 0), max(0, 0, 0) {}
        AABB(const Vector3& min, const Vector3& max) : min(min), max(max) {}

        // Center 계산
        Vector3 Center() const {
            return (min + max) * 0.5f;
        }

        // Size 계산
        Vector3 Size() const {
            return Vector3(max.x - min.x, max.y - min.y, max.z - min.z);
        }

        // 교차 테스트
        bool Intersects(const AABB& other) const {
            return (min.x <= other.max.x && max.x >= other.min.x) &&
                   (min.y <= other.max.y && max.y >= other.min.y) &&
                   (min.z <= other.max.z && max.z >= other.min.z);
        }

        // 점 포함 테스트
        bool Contains(const Vector3& point) const {
            return (point.x >= min.x && point.x <= max.x) &&
                   (point.y >= min.y && point.y <= max.y) &&
                   (point.z >= min.z && point.z <= max.z);
        }

        // 합집합 AABB 생성
        static AABB Union(const AABB& a, const AABB& b) {
            Vector3 min(
                std::fmin(a.min.x, b.min.x),
                std::fmin(a.min.y, b.min.y),
                std::fmin(a.min.z, b.min.z)
            );
            
            Vector3 max(
                std::fmax(a.max.x, b.max.x),
                std::fmax(a.max.y, b.max.y),
                std::fmax(a.max.z, b.max.z)
            );
            
            return AABB(min, max);
        }
    };
}