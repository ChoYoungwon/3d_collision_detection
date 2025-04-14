#ifndef GJK_H
#define GJK_H

#include "../math/Vector3.h"
#include "../decomposition/ConvexHull.h"
#include <vector>

namespace Collision {

    class GJK {
    public:
        GJK() {}

        // 두 볼록체(ConvexHull)의 충돌 여부 판단 함수
        bool Intersect(
            const ConvexHull& shapeA, 
            const ConvexHull& shapeB,
            const Vector3& posA, 
            const Vector3& posB
        );

        bool DoSimplex(std::vector<Vector3>& simplex, Vector3& direction);

        bool doLine(std::vector<Vector3>& simplex, Vector3& direction);

        bool doTriangle(std::vector<Vector3>& simplex, Vector3& direction);

        bool doTetrahedron(std::vector<Vector3>& simplex, Vector3& direction);

        // 두 객체의 Minkowski 차 집합에서 주어진 방향의 지원 점을 반환
        Vector3 Support(const ConvexHull& shapeA, const ConvexHull& shapeB, 
            const Vector3& dir, const Vector3& posA, const Vector3& posB);

    private:

        Vector3 getFarthestPointInDirection(const ConvexHull& shape, 
            const Vector3& dir, 
            const Vector3& position
        );

        // 현재 단순체(simplex)를 분석하여 원점을 포함하는지 판단하고,
        // 그렇지 않다면 새로운 검색 방향을 업데이트합니다.
        bool HandleSimplex(std::vector<Vector3>& simplex, Vector3& direction);

        // 원점과 심플렉스 간 거리 계산 함수 추가
        float calculateDistanceToOrigin(const std::vector<Vector3>& simplex);
    };

} // namespace Collision

#endif // GJK_H
