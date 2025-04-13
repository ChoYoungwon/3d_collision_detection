#include "SAT.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace Collision {

    // OverlapOnAxis: 주어진 축(axis)에 대해 OBB의 모든 정점(vertex)을 투영하여 최소/최대 값을 계산하고, 두 구간이 겹치는지 체크
    bool SAT::OverlapOnAxis(const OBB& obbA, const OBB& obbB, const Vector3& axis) {
        Vector3 axisNorm = axis.Normalized();

        // 람다 함수: OBB의 모든 정점을 투영하여 [min, max] 구간을 반환
        auto getProjection = [&](const OBB& obb) -> std::pair<float, float> {
            float minProj = std::numeric_limits<float>::infinity();
            float maxProj = -std::numeric_limits<float>::infinity();
            for (const Vector3& vertex : obb.GetVertices()) { // OBB 클래스는 8개의 vertex를 반환
                float proj = vertex.Dot(axisNorm);
                minProj = std::min(minProj, proj);
                maxProj = std::max(maxProj, proj);
            }
            return std::make_pair(minProj, maxProj);
        };

        auto [minA, maxA] = getProjection(obbA);
        auto [minB, maxB] = getProjection(obbB);

        // 구간이 겹치는지 확인 (한쪽 끝이 다른 구간의 시작보다 뒤에 있는지 확인)
        return (maxA >= minB) && (maxB >= minA);
    }

    bool SAT::TestOBBCollision(const OBB& obbA, const OBB& obbB) {
        std::vector<Vector3> axes;
        // OBB가 제공하는 지역 좌표계의 축 (보통 3개: X, Y, Z)
        const auto& axesA = obbA.GetAxes(); // 예: std::array<Vector3, 3>
        const auto& axesB = obbB.GetAxes();

        // 1. 각 OBB의 면의 법선(축)을 후보 축에 추가
        axes.insert(axes.end(), axesA.begin(), axesA.end());
        axes.insert(axes.end(), axesB.begin(), axesB.end());

        // 2. 각 OBB의 에지들 사이의 교차 벡터 계산 (예: 3x3 = 9개의 후보 축)
        for (const auto& a : axesA) {
            for (const auto& b : axesB) {
                Vector3 crossAxis = a.Cross(b);
                // 벡터가 거의 0이면(평행한 경우) 무시
                if (crossAxis.LengthSquared() < 1e-6) continue;
                axes.push_back(crossAxis.Normalized());
            }
        }

        // 모든 후보 축에 대해 두 OBB의 투영 구간을 검사
        for (const auto& axis : axes) {
            if (!OverlapOnAxis(obbA, obbB, axis)) {
                // 한 축에서라도 분리됨이 확인되면 충돌하지 않음
                return false;
            }
        }

        // 모든 축에서 투영이 겹치면 충돌로 판단
        return true;
    }

} // namespace Collision