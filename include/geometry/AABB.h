#ifndef AABB_H
#define AABB_H

#include "Vector3.h"
#include <vector>
#include <limits>
#include <algorithm>

// Axis-Aligned Bounding Box (축 정렬 경계 상자)
class AABB {
public:
    Vector3 min;  // 최소점 (x_min, y_min, z_min)
    Vector3 max;  // 최대점 (x_max, y_max, z_max)

    // 기본 생성자 - 유효하지 않은 AABB 생성
    AABB() 
        : min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
          max(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()) {
    }

    // 최소/최대점으로 초기화하는 생성자
    AABB(const Vector3& minPoint, const Vector3& maxPoint) 
        : min(minPoint), max(maxPoint) {
    }

    // 정점 배열로 초기화하는 생성자
    AABB(const std::vector<Vector3>& points) {
        computeFromPoints(points);
    }

    // 정점 배열로부터 AABB 계산
    void computeFromPoints(const std::vector<Vector3>& points);

    // 중심점 반환
    Vector3 getCenter() const {
        return (min + max) * 0.5f;
    }

    // 크기(width, height, depth) 반환
    Vector3 getSize() const {
        return max - min;
    }

    // 체적 반환
    float getVolume() const {
        Vector3 size = getSize();
        return size.x * size.y * size.z;
    }

    // 다른 AABB와 교차 여부 확인
    bool intersects(const AABB& other) const {
        return (min.x <= other.max.x && max.x >= other.min.x) &&
               (min.y <= other.max.y && max.y >= other.min.y) &&
               (min.z <= other.max.z && max.z >= other.min.z);
    }

    // 점이 AABB 내부에 있는지 확인
    bool contains(const Vector3& point) const {
        return (point.x >= min.x && point.x <= max.x) &&
               (point.y >= min.y && point.y <= max.y) &&
               (point.z >= min.z && point.z <= max.z);
    }

    // 다른 AABB를 포함하는지 확인
    bool contains(const AABB& other) const {
        return (min.x <= other.min.x && max.x >= other.max.x) &&
               (min.y <= other.min.y && max.y >= other.max.y) &&
               (min.z <= other.min.z && max.z >= other.max.z);
    }

    // 다른 AABB와 합집합
    AABB merge(const AABB& other) const {
        return AABB(
            Vector3(
                std::min(min.x, other.min.x),
                std::min(min.y, other.min.y),
                std::min(min.z, other.min.z)
            ),
            Vector3(
                std::max(max.x, other.max.x),
                std::max(max.y, other.max.y),
                std::max(max.z, other.max.z)
            )
        );
    }

    // 다른 AABB와 교집합
    AABB intersection(const AABB& other) const {
        return AABB(
            Vector3(
                std::max(min.x, other.min.x),
                std::max(min.y, other.min.y),
                std::max(min.z, other.min.z)
            ),
            Vector3(
                std::min(max.x, other.max.x),
                std::min(max.y, other.max.y),
                std::min(max.z, other.max.z)
            )
        );
    }

    // 유효한 AABB인지 확인 (최소점이 최대점보다 작거나 같은지)
    bool isValid() const {
        return min.x <= max.x && min.y <= max.y && min.z <= max.z;
    }
};

#endif // AABB_H