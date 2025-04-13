#ifndef GJK_H
#define GJK_H

#include "../math/Vector3.h"
#include "../geometry/ConvexHull.h"
#include <vector>

namespace Collision {

    class GJK {
    public:
        GJK() {}
        // 두 볼록체(ConvexHull)의 충돌 여부 판단 함수
        bool Intersect(const ConvexHull& shapeA, const ConvexHull& shapeB);

    private:
        // 두 객체의 Minkowski 차 집합에서 주어진 방향의 지원 점을 반환
        Vector3 Support(const ConvexHull& shapeA, const ConvexHull& shapeB, const Vector3& dir);
        // 현재 단순체(simplex)를 분석하여 원점을 포함하는지 판단하고,
        // 그렇지 않다면 새로운 검색 방향을 업데이트합니다.
        bool HandleSimplex(std::vector<Vector3>& simplex, Vector3& direction);
    };

} // namespace Collision

#endif // GJK_H
