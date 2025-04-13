// include/geometry/AABB.h
#ifndef AABB_H
#define AABB_H

#include "../math/Vector3.h"
#include <algorithm>
#include <limits>
#include <vector>

class AABB {
public:
    Vector3 min; // 최소 경계점
    Vector3 max; // 최대 경계점

    // 생성자
    AABB() :
        min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
        max(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()) {}

    AABB(const Vector3& min, const Vector3& max) : min(min), max(max) {}

    // 점들의 집합으로부터 AABB 생성
    AABB(const std::vector<Vector3>& points) {
        if (points.empty()) {
            min = Vector3(0, 0, 0);
            max = Vector3(0, 0, 0);
            return;
        }

        min = Vector3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        max = Vector3(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

        for (const auto& point : points) {
            min.x = std::min(min.x, point.x);
            min.y = std::min(min.y, point.y);
            min.z = std::min(min.z, point.z);

            max.x = std::max(max.x, point.x);
            max.y = std::max(max.y, point.y);
            max.z = std::max(max.z, point.z);
        }
    }

    // 크기 계산
    Vector3 getSize() const {
        return max - min;
    }

    // 중심점 계산
    Vector3 getCenter() const {
        return (min + max) * 0.5f;
    }

    Vector3 getExtents() const;


    // AABB 확장
    void expand(const Vector3& point) {
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        min.z = std::min(min.z, point.z);

        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
        max.z = std::max(max.z, point.z);
    }

    void expand(const AABB& other) {
        min.x = std::min(min.x, other.min.x);
        min.y = std::min(min.y, other.min.y);
        min.z = std::min(min.z, other.min.z);

        max.x = std::max(max.x, other.max.x);
        max.y = std::max(max.y, other.max.y);
        max.z = std::max(max.z, other.max.z);
    }

    // 점이 AABB 내부에 있는지 확인
    bool contains(const Vector3& point) const {
        return point.x >= min.x && point.x <= max.x &&
            point.y >= min.y && point.y <= max.y &&
            point.z >= min.z && point.z <= max.z;
    }

    // AABB가 다른 AABB를 완전히 포함하는지 확인
    bool contains(const AABB& other) const {
        return min.x <= other.min.x && max.x >= other.max.x &&
            min.y <= other.min.y && max.y >= other.max.y &&
            min.z <= other.min.z && max.z >= other.max.z;
    }

    // AABB 충돌 검사
    bool intersects(const AABB& other) const {
        return max.x >= other.min.x && min.x <= other.max.x &&
            max.y >= other.min.y && min.y <= other.max.y &&
            max.z >= other.min.z && min.z <= other.max.z;
    }

    // 두 AABB의 교집합 계산
    AABB getIntersection(const AABB& other) const {
        Vector3 intersectionMin(
            std::max(min.x, other.min.x),
            std::max(min.y, other.min.y),
            std::max(min.z, other.min.z)
        );

        Vector3 intersectionMax(
            std::min(max.x, other.max.x),
            std::min(max.y, other.max.y),
            std::min(max.z, other.max.z)
        );

        // 교집합이 없으면 빈 AABB 반환
        if (intersectionMin.x > intersectionMax.x ||
            intersectionMin.y > intersectionMax.y ||
            intersectionMin.z > intersectionMax.z) {
            return AABB();
        }

        return AABB(intersectionMin, intersectionMax);
    }

    // 체적 계산
    float getVolume() const {
        Vector3 size = getSize();
        return size.x * size.y * size.z;
    }

    // 표면적 계산
    float getSurfaceArea() const {
        Vector3 size = getSize();
        return 2.0f * (size.x * size.y + size.x * size.z + size.y * size.z);
    }
};

#endif // AABB_H