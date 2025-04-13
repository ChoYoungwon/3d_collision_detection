#include "SAT.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace Collision {

    // OverlapOnAxis: 주어진 축(axis)에 대해, OBB의 모든 정점(projection)을 구한 후 겹침 여부를 판단
    bool SAT::OverlapOnAxis(const OBB& obbA, const OBB& obbB, const Vector3& axis) {
        // 주어진 축을 단위 벡터로 만듭니다.
        Vector3 axisNorm = axis.normalized();

        // 람다 함수: 해당 OBB의 모든 정점을 주어진 축에 투영하여 [min, max] 범위를 반환
        auto getProjection = [&](const OBB& obb) -> std::pair<float, float> {
            float minProj = std::numeric_limits<float>::infinity();
            float maxProj = -std::numeric_limits<float>::infinity();
            // OBB의 정점은 getCorners() 함수로 구합니다.
            for (const Vector3& vertex : obb.getCorners()) {
                float proj = vertex.dot(axisNorm);
                minProj = std::min(minProj, proj);
                maxProj = std::max(maxProj, proj);
            }
            return std::make_pair(minProj, maxProj);
        };

        auto [minA, maxA] = getProjection(obbA);
        auto [minB, maxB] = getProjection(obbB);

        // 두 프로젝션 구간이 겹치는지 검사
        return (maxA >= minB) && (maxB >= minA);
    }

    // TestOBBCollision: 분리 축 정리(SAT)를 사용하여 두 OBB의 충돌 여부를 검사합니다.
    bool SAT::TestOBBCollision(const OBB& obbA, const OBB& obbB) {
        std::vector<Vector3> axes;
        // 각 OBB의 로컬 축(회전 행렬의 열 벡터)을 구합니다.
        const auto& axesA = obbA.getAxes();
        const auto& axesB = obbB.getAxes();

        // 두 OBB의 축들을 축 목록에 추가합니다.
        axes.insert(axes.end(), axesA.begin(), axesA.end());
        axes.insert(axes.end(), axesB.begin(), axesB.end());

        // 두 OBB의 축들의 외적에 의해 생성되는 분리 축들을 추가 (0 벡터는 건너뜁니다.)
        for (const auto& a : axesA) {
            for (const auto& b : axesB) {
                Vector3 crossAxis = a.cross(b);
                if (crossAxis.magnitudeSquared() < 1e-6f) continue;
                axes.push_back(crossAxis.normalized());
            }
        }

        // 각 축에 대해 두 OBB의 투영 구간이 겹치는지 검사
        for (const auto& axis : axes) {
            if (!OverlapOnAxis(obbA, obbB, axis)) {
                // 어느 한 축이라도 분리 축이 존재하면 충돌하지 않습니다.
                return false;
            }
        }

        // 모든 축에서 투영 구간이 겹치면 두 OBB는 충돌합니다.
        return true;
    }

} // namespace Collision