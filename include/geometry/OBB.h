#ifndef OBB_H
#define OBB_H

#include "../math/Vector3.h"
#include "../math/Matrix3x3.h"
#include "../math/Quaternion.h"
#include "AABB.h"
#include <vector>
#include <array>
#include <cmath>  // std::abs

class OBB {
public:
    Vector3 center;       // 중심 좌표
    Vector3 halfExtents;  // 각 축에 대한 반 크기
    Matrix3x3 orientation; // 회전 행렬

    // 기본 생성자: 중심 (0,0,0), 반 크기 (1,1,1), 단위 행렬
    OBB() : center(0, 0, 0), halfExtents(1, 1, 1), orientation() {}

    // 중심과 반 크기만 지정 (회전은 단위 행렬)
    OBB(const Vector3& center, const Vector3& halfExtents)
        : center(center), halfExtents(halfExtents), orientation() {}

    // 중심, 반 크기, 회전 행렬을 지정
    OBB(const Vector3& center, const Vector3& halfExtents, const Matrix3x3& orientation)
        : center(center), halfExtents(halfExtents), orientation(orientation) {}

    // AABB로부터 OBB 생성
    OBB(const AABB& aabb) {
        center = aabb.getCenter();
        halfExtents = aabb.getSize() * 0.5f;
        orientation = Matrix3x3(); // 단위 행렬
    }

    // 점들의 집합으로부터 OBB 생성 (먼저 AABB로 감싸고 사용)
    OBB(const std::vector<Vector3>& points) {
        if (points.empty()) {
            center = Vector3(0, 0, 0);
            halfExtents = Vector3(0, 0, 0);
            orientation = Matrix3x3();
            return;
        }
        AABB aabb(points);
        center = aabb.getCenter();
        halfExtents = aabb.getSize() * 0.5f;
        orientation = Matrix3x3();
    }

    // OBB의 8개 모서리(월드 좌표) 반환
    std::array<Vector3, 8> getCorners() const {
        std::array<Vector3, 8> corners;
        corners[0] = Vector3(-halfExtents.x, -halfExtents.y, -halfExtents.z);
        corners[1] = Vector3( halfExtents.x, -halfExtents.y, -halfExtents.z);
        corners[2] = Vector3( halfExtents.x,  halfExtents.y, -halfExtents.z);
        corners[3] = Vector3(-halfExtents.x,  halfExtents.y, -halfExtents.z);
        corners[4] = Vector3(-halfExtents.x, -halfExtents.y,  halfExtents.z);
        corners[5] = Vector3( halfExtents.x, -halfExtents.y,  halfExtents.z);
        corners[6] = Vector3( halfExtents.x,  halfExtents.y,  halfExtents.z);
        corners[7] = Vector3(-halfExtents.x,  halfExtents.y,  halfExtents.z);
        
        for (auto& corner : corners) {
            corner = orientation * corner + center;
        }
        return corners;
    }

    // OBB의 로컬 좌표계 축(회전 행렬의 열 벡터) 반환
    std::array<Vector3, 3> getAxes() const {
        std::array<Vector3, 3> axes;
        axes[0] = Vector3(orientation(0, 0), orientation(1, 0), orientation(2, 0));
        axes[1] = Vector3(orientation(0, 1), orientation(1, 1), orientation(2, 1));
        axes[2] = Vector3(orientation(0, 2), orientation(1, 2), orientation(2, 2));
        return axes;
    }

    // 편의 함수: i번째 축 반환 (i = 0, 1, 2)
    Vector3 getAxis(int i) const;

    // OBB를 감싸는 AABB 반환
    AABB toAABB() const {
        std::array<Vector3, 8> corners = getCorners();
        return AABB(std::vector<Vector3>(corners.begin(), corners.end()));
    }

    // 주어진 점이 OBB 내부에 있는지 검사 (OBB 좌표계로 변환 후 비교)
    bool contains(const Vector3& point) const {
        Vector3 localPoint = orientation.transpose() * (point - center);
        return (std::abs(localPoint.x) <= halfExtents.x &&
                std::abs(localPoint.y) <= halfExtents.y &&
                std::abs(localPoint.z) <= halfExtents.z);
    }

    // 분리 축 정리(SAT)을 통해 두 OBB가 교차하는지 검사
    bool intersects(const OBB& other) const {
        std::array<Vector3, 3> axesA = getAxes();
        std::array<Vector3, 3> axesB = other.getAxes();
        Vector3 t = other.center - center;
        
        // A의 축에 대해 검사
        for (int i = 0; i < 3; i++) {
            if (!overlapOnAxis(other, axesA[i], t))
                return false;
        }
        // B의 축에 대해 검사
        for (int i = 0; i < 3; i++) {
            if (!overlapOnAxis(other, axesB[i], t))
                return false;
        }
        // 두 축의 외적으로 만들어진 분리 축에 대해 검사
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Vector3 axis = axesA[i].cross(axesB[j]);
                if (axis.dot(axis) < 1e-6f)  // 벡터의 길이 제곱 검사
                    continue;
                axis.normalize();
                if (!overlapOnAxis(other, axis, t))
                    return false;
            }
        }
        return true;
    }

    // 정적 함수: 최소 좌표와 최대 좌표로부터 AABB를 만들고, 이를 이용해 OBB 생성
    static OBB fromAABB(const Vector3& min, const Vector3& max);

private:
    // 주어진 축(axis)에 대해 현재 OBB와 다른 OBB가 겹치는지 검사
    bool overlapOnAxis(const OBB& other, const Vector3& axis, const Vector3& t) const {
        float ra = projectExtent(axis);
        float rb = other.projectExtent(axis);
        float t_projection = std::abs(t.dot(axis));
        return t_projection <= (ra + rb);
    }

    // 주어진 축(axis)에 대해, 현재 OBB의 투영(반 길이) 합산
    float projectExtent(const Vector3& axis) const {
        std::array<Vector3, 3> axes = getAxes();
        // halfExtents의 각 축값과 해당 방향 벡터를 곱해 dot 연산으로 투영 길이 계산
        return std::abs((this->halfExtents.x * axes[0]).dot(axis)) +
               std::abs((this->halfExtents.y * axes[1]).dot(axis)) +
               std::abs((this->halfExtents.z * axes[2]).dot(axis));
    }
};

#endif // OBB_H
