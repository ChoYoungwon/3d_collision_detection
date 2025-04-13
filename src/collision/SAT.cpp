#include "SAT.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace Collision {

    // OverlapOnAxis: �־��� ��(axis)�� ���� OBB�� ��� ����(vertex)�� �����Ͽ� �ּ�/�ִ� ���� ����ϰ�, �� ������ ��ġ���� üũ
    bool SAT::OverlapOnAxis(const OBB& obbA, const OBB& obbB, const Vector3& axis) {
        Vector3 axisNorm = axis.Normalized();

        // ���� �Լ�: OBB�� ��� ������ �����Ͽ� [min, max] ������ ��ȯ
        auto getProjection = [&](const OBB& obb) -> std::pair<float, float> {
            float minProj = std::numeric_limits<float>::infinity();
            float maxProj = -std::numeric_limits<float>::infinity();
            for (const Vector3& vertex : obb.GetVertices()) { // OBB Ŭ������ 8���� vertex�� ��ȯ
                float proj = vertex.Dot(axisNorm);
                minProj = std::min(minProj, proj);
                maxProj = std::max(maxProj, proj);
            }
            return std::make_pair(minProj, maxProj);
        };

        auto [minA, maxA] = getProjection(obbA);
        auto [minB, maxB] = getProjection(obbB);

        // ������ ��ġ���� Ȯ�� (���� ���� �ٸ� ������ ���ۺ��� �ڿ� �ִ��� Ȯ��)
        return (maxA >= minB) && (maxB >= minA);
    }

    bool SAT::TestOBBCollision(const OBB& obbA, const OBB& obbB) {
        std::vector<Vector3> axes;
        // OBB�� �����ϴ� ���� ��ǥ���� �� (���� 3��: X, Y, Z)
        const auto& axesA = obbA.GetAxes(); // ��: std::array<Vector3, 3>
        const auto& axesB = obbB.GetAxes();

        // 1. �� OBB�� ���� ����(��)�� �ĺ� �࿡ �߰�
        axes.insert(axes.end(), axesA.begin(), axesA.end());
        axes.insert(axes.end(), axesB.begin(), axesB.end());

        // 2. �� OBB�� ������ ������ ���� ���� ��� (��: 3x3 = 9���� �ĺ� ��)
        for (const auto& a : axesA) {
            for (const auto& b : axesB) {
                Vector3 crossAxis = a.Cross(b);
                // ���Ͱ� ���� 0�̸�(������ ���) ����
                if (crossAxis.LengthSquared() < 1e-6) continue;
                axes.push_back(crossAxis.Normalized());
            }
        }

        // ��� �ĺ� �࿡ ���� �� OBB�� ���� ������ �˻�
        for (const auto& axis : axes) {
            if (!OverlapOnAxis(obbA, obbB, axis)) {
                // �� �࿡���� �и����� Ȯ�εǸ� �浹���� ����
                return false;
            }
        }

        // ��� �࿡�� ������ ��ġ�� �浹�� �Ǵ�
        return true;
    }

} // namespace Collision