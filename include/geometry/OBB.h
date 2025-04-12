// include/geometry/OBB.h
#ifndef OBB_H
#define OBB_H

#include "../math/Vector3.h"
#include "../math/Matrix3x3.h"
#include "../math/Quaternion.h"
#include "AABB.h"
#include <vector>
#include <array>

class OBB {
public:
    Vector3 center;      // 중심점
    Vector3 halfExtents; // 각 축 방향으로의 절반 크기
    Matrix3x3 orientation; // 방향 (회전 행렬)

    // 생성자
    OBB() : center(0, 0, 0), halfExtents(1, 1, 1) {
        // 기본 방향은 항등 행렬 (축 정렬)
        orientation = Matrix3x3();
    }

    OBB(const Vector3& center, const Vector3& halfExtents)
        : center(center), halfExtents(halfExtents) {
        // 기본 방향은 항등 행렬 (축 정렬)
        orientation = Matrix3x3();
    }

    OBB(const Vector3& center, const Vector3& halfExtents, const Matrix3x3& orientation)
        : center(center), halfExtents(halfExtents), orientation(orientation) {
    }

    // AABB로부터 OBB 생성
    OBB(const AABB& aabb) {
        center = aabb.getCenter();
        halfExtents = aabb.getSize() * 0.5f;
        orientation = Matrix3x3(); // 항등 행렬 (축 정렬)
    }

    // 점들의 집합으로부터 OBB 생성 (주성분 분석 방법)
    // 간단한 구현을 위해 축 정렬 방식 사용
    OBB(const std::vector<Vector3>& points) {
        if (points.empty()) {
            center = Vector3(0, 0, 0);
            halfExtents = Vector3(0, 0, 0);
            orientation = Matrix3x3();
            return;
        }

        // 주성분 분석은 복잡하므로 여기서는 간단히 AABB로 변환
        AABB aabb(points);
        center = aabb.getCenter();
        halfExtents = aabb.getSize() * 0.5f;
        orientation = Matrix3x3(); // 항등 행렬 (축 정렬)
    }

    // OBB의 8개 꼭짓점 구하기
    std::array<Vector3, 8> getCorners() const {
        std::array<Vector3, 8> corners;

        // 로컬 공간의 꼭짓점 계산
        corners[0] = Vector3(-halfExtents.x, -halfExtents.y, -halfExtents.z);
        corners[1] = Vector3(halfExtents.x, -halfExtents.y, -halfExtents.z);
        corners[2] = Vector3(halfExtents.x, halfExtents.y, -halfExtents.z);
        corners[3] = Vector3(-halfExtents.x, halfExtents.y, -halfExtents.z);
        corners[4] = Vector3(-halfExtents.x, -halfExtents.y, halfExtents.z);
        corners[5] = Vector3(halfExtents.x, -halfExtents.y, halfExtents.z);
        corners[6] = Vector3(halfExtents.x, halfExtents.y, halfExtents.z);
        corners[7] = Vector3(-halfExtents.x, halfExtents.y, halfExtents.z);

        // 월드 공간으로 변환
        for (auto& corner : corners) {
            corner = orientation * corner + center;
        }

        return corners;
    }

    // OBB의 축 구하기
    std::array<Vector3, 3> getAxes() const {
        std::array<Vector3, 3> axes;

        // 각 행이 축을 나타냄
        axes[0] = Vector3(orientation(0, 0), orientation(0, 1), orientation(0, 2));
        axes[1] = Vector3(orientation(1, 0), orientation(1, 1), orientation(1, 2));
        axes[2] = Vector3(orientation(2, 0), orientation(2, 1), orientation(2, 2));

        return axes;
    }

    // OBB를 AABB로 변환 (경계를 포함하는 가장 작은 AABB)
    AABB toAABB() const {
        std::array<Vector3, 8> corners = getCorners();
        return AABB(std::vector<Vector3>(corners.begin(), corners.end()));
    }

    // 점이 OBB 내부에 있는지 확인
    bool contains(const Vector3& point) const {
        // 점을 OBB의 로컬 공간으로 변환
        Vector3 localPoint = orientation.transpose() * (point - center);

        // 로컬 공간에서 범위 확인
        return std::abs(localPoint.x) <= halfExtents.x &&
            std::abs(localPoint.y) <= halfExtents.y &&
            std::abs(localPoint.z) <= halfExtents.z;
    }

    // OBB 충돌 검사 (분리축 정리 사용)
    bool intersects(const OBB& other) const {
        // 15개의 분리축 테스트 필요
        // - 두 OBB의 각 3개 축 (총 6개)
        // - 두 OBB 축의 외적 (총 9개)

        // OBB A의 축
        std::array<Vector3, 3> axesA = getAxes();
        // OBB B의 축
        std::array<Vector3, 3> axesB = other.getAxes();

        // OBB의 중심 간 벡터
        Vector3 t = other.center - center;

        // 각 OBB의 축으로 테스트
        for (int i = 0; i < 3; i++) {
            if (!overlapOnAxis(other, axesA[i], t)) {
                return false;
            }
        }

        for (int i = 0; i < 3; i++) {
            if (!overlapOnAxis(other, axesB[i], t)) {
                return false;
            }
        }

        // 두 OBB의 축 간 외적으로 테스트
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Vector3 axis = Vector3::cross(axesA[i], axesB[j]);
                // 외적이 0이면 축이 평행하므로 무시
                if (axis.lengthSquared() < 1e-6f) {
                    continue;
                }
                axis.normalize();

                if (!overlapOnAxis(other, axis, t)) {
                    return false;
                }
            }
        }

        // 모든 축에서 겹침이 있으면 충돌
        return true;
    }

private:
    // 특정 축에 대한 OBB 투영 겹침 검사
    bool overlapOnAxis(const OBB& other, const Vector3& axis, const Vector3& t) const {
        // 각 OBB의 축에 대한 반투영길이 계산
        float ra = projectExtent(axis);
        float rb = other.projectExtent(axis);

        // 중심 간 거리의 투영 계산
        float t_projection = std::abs(Vector3::dot(t, axis));

        // 축에 대한 겹침 검사
        return t_projection <= (ra + rb);
    }

    // OBB의 반투영길이 계산
    float projectExtent(const Vector3& axis) const {
        // 각 로컬 축에 대한 투영 합산
        std::array<Vector3, 3> obb_axes = getAxes();
        return std::abs(Vector3::dot(halfExtents.x * obb_axes[0], axis)) +
            std::abs(Vector3::dot(halfExtents.y * obb_axes[1], axis)) +
            std::abs(Vector3::dot(halfExtents.z * obb_axes[2], axis));
    }
};

#endif // OBB_H