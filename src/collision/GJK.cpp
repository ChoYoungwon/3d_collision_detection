#include "GJK.h"
#include <cmath>
#include <algorithm>

namespace Collision {

    // shapeA와 shapeB의 support 함수를 사용하여 Minkowski 차 집합의 점을 반환합니다.
    Vector3 GJK::Support(const ConvexHull& shapeA, const ConvexHull& shapeB, const Vector3& dir) {
        // shapeA의 dir 방향 최대로 멀리 있는 점
        Vector3 pointA = shapeA.support(dir);
        // shapeB의 -dir 방향 최대로 멀리 있는 점
        Vector3 pointB = shapeB.support(-dir);
        return pointA - pointB;
    }

    bool GJK::Intersect(const ConvexHull& shapeA, const ConvexHull& shapeB) {
        // 초기 방향: (1, 0, 0)
        Vector3 direction(1, 0, 0);
        std::vector<Vector3> simplex;
        simplex.push_back(Support(shapeA, shapeB, direction));
        // 새로운 검색 방향: 원점 반대 방향
        direction = -simplex[0];

        // 반복적으로 단순체(simplex)를 확장하면서 원점을 포함하는지 검사합니다.
        while (true) {
            Vector3 newPoint = Support(shapeA, shapeB, direction);
            // 새 점이 현재 방향으로 원점을 향하지 않는다면 충돌이 없음
            if (newPoint.dot(direction) <= 0) {
                return false;
            }
            simplex.push_back(newPoint);
            if (HandleSimplex(simplex, direction)) {
                return true;
            }
        }
    }

    // HandleSimplex 함수: 현재 단순체의 모양(line, triangle, tetrahedron)에 따라
    // 원점 포함 여부를 검사하고, 그렇지 않으면 새로운 탐색 방향을 결정합니다.
    bool GJK::HandleSimplex(std::vector<Vector3>& simplex, Vector3& direction) {
        if (simplex.size() == 2) {
            // ---------- 선분(simplex = {A, B}) ----------
            // A: 마지막에 추가된 점, B: 처음 추가된 점
            Vector3 A = simplex.back();
            Vector3 B = simplex.front();
            Vector3 AB = B - A;
            Vector3 AO = -A; // A에서 원점까지의 벡터

            // AB와 AO의 평면에 수직인 방향(AB cross (AO cross AB))를 새로운 탐색 방향으로 설정
            direction = AB.cross(AO).cross(AB);
            if (direction.magnitudeSquared() < 1e-6f) {
                direction = Vector3(-AB.y, AB.x, 0);
            }
            return false;
        }
        else if (simplex.size() == 3) {
            // ---------- 삼각형(simplex = {C, B, A}) ----------
            // A: 마지막에 추가된 점, B와 C는 순서대로 이전 점들
            Vector3 A = simplex[2];
            Vector3 B = simplex[1];
            Vector3 C = simplex[0];
            Vector3 AB = B - A;
            Vector3 AC = C - A;
            Vector3 AO = -A;

            // 삼각형의 법선
            Vector3 ABC = AB.cross(AC);

            // Edge AB에 수직인 평면 (ABC cross AB)
            Vector3 ABPerp = ABC.cross(AB);
            if (ABPerp.dot(AO) > 0) {
                // C를 단순체에서 제거하고, 새로운 방향으로 AB의 방향 설정
                simplex.erase(simplex.begin()); // C 제거
                direction = AB.cross(AO).cross(AB);
                return false;
            }

            // Edge AC에 수직인 평면 (AC cross ABC)
            Vector3 ACPerp = AC.cross(ABC);
            if (ACPerp.dot(AO) > 0) {
                // B를 단순체에서 제거하고, 새로운 방향으로 AC의 방향 설정
                simplex.erase(simplex.begin() + 1); // B 제거
                direction = AC.cross(AO).cross(AC);
                return false;
            }

            // 만약 위 두 경우가 아니라면,
            // 원점이 삼각형 내부의 영역에 있으면 삼각형의 법선 방향을 탐색 방향으로 설정
            if (ABC.dot(AO) > 0) {
                direction = ABC;
            }
            else {
                // 그렇지 않으면 법선의 반대 방향을 탐색 방향으로 사용하고,
                // simplex의 순서를 교환하여 일관성을 유지합니다.
                direction = -ABC;
                std::swap(simplex[0], simplex[1]);
            }
            return false;
        }
        else if (simplex.size() == 4) {
            // ---------- 사면체(simplex = {D, C, B, A}) ----------
            // A: 마지막에 추가된 점, B, C, D는 순서대로 이전 점들
            Vector3 A = simplex[3];
            Vector3 B = simplex[2];
            Vector3 C = simplex[1];
            Vector3 D = simplex[0];
            Vector3 AO = -A;

            Vector3 ABC = (B - A).cross(C - A);
            Vector3 ACD = (C - A).cross(D - A);
            Vector3 ADB = (D - A).cross(B - A);

            // 각각의 면에 대해 원점이 같은 쪽에 있는지 검사합니다.
            if (ABC.dot(AO) > 0) {
                simplex.erase(simplex.begin()); // D 제거
                direction = ABC;
                return false;
            }
            if (ACD.dot(AO) > 0) {
                simplex.erase(simplex.begin() + 2); // B 제거
                direction = ACD;
                return false;
            }
            if (ADB.dot(AO) > 0) {
                simplex.erase(simplex.begin() + 1); // C 제거
                direction = ADB;
                return false;
            }
            // 모든 면에 대해 원점이 내부에 있다면 두 물체는 충돌 중입니다.
            return true;
        }
        return false;
    }

} // namespace Collision
