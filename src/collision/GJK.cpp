#include "GJK.h"
#include <cmath>
#include <algorithm>

namespace Collision {

    // 지원 함수: 두 객체에서 각각 주어진 방향과 반대 방향의 최대 거리를 가진 점을 구한 후, 두 점의 차(= Minkowski 차)를 반환
    Vector3 GJK::Support(const ConvexHull& shapeA, const ConvexHull& shapeB, const Vector3& dir) {
        // shapeA에서 방향 dir로 가장 먼 점
        Vector3 pointA = shapeA.GetFurthestPoint(dir);
        // shapeB에서는 -dir 방향으로 가장 먼 점 (즉, 반대쪽)
        Vector3 pointB = shapeB.GetFurthestPoint(-dir);
        return pointA - pointB;
    }

    bool GJK::Intersect(const ConvexHull& shapeA, const ConvexHull& shapeB) {
        // 1. 초기 방향 선택 (임의의 벡터)
        Vector3 direction(1, 0, 0);
        // 2. 최초의 지원 점을 단순체에 추가
        std::vector<Vector3> simplex;
        simplex.push_back(Support(shapeA, shapeB, direction));
        // 3. 원점을 향하는 검색 방향: 단순체의 첫 점의 반대 방향
        direction = -simplex[0];

        // 4. 반복 루프: 새로운 지원 점을 계산해 단순체에 추가하면서 원점 포함 여부를 판단
        while (true) {
            Vector3 newPoint = Support(shapeA, shapeB, direction);
            // 만약 새 지원 점이 진행 방향에서 원점을 넘어가지 못하면 충돌하지 않는 것으로 판단
            if (newPoint.Dot(direction) <= 0) {
                return false;
            }
            // 단순체에 새 점 추가
            simplex.push_back(newPoint);
            // 단순체를 분석하여 (업데이트) 원점이 포함되었는지 확인
            if (HandleSimplex(simplex, direction)) {
                return true;
            }
        }
    }

    // HandleSimplex 함수: 단순체의 점 개수에 따라 세 가지 경우(line, triangle, tetrahedron)를 처리
    bool GJK::HandleSimplex(std::vector<Vector3>& simplex, Vector3& direction) {
        if (simplex.size() == 2) {
            // ---------- 선분(simplex = {A, B}) 처리 ----------
            // 단순체의 마지막에 추가된 점을 A, 그 이전 점을 B로 간주합니다.
            Vector3 A = simplex.back();
            Vector3 B = simplex.front();
            Vector3 AB = B - A;
            Vector3 AO = -A; // 원점으로 향하는 벡터

            // A와 AB 선분에 수직인(원점을 향하는) 벡터를 찾음
            direction = AB.Cross(AO).Cross(AB);
            // 만약 direction이 0 벡터라면, AB에 수직인 임의의 벡터를 선택
            if (direction.LengthSquared() < 1e-6f) {
                direction = Vector3(-AB.y, AB.x, 0);
            }
        }
        else if (simplex.size() == 3) {
            // ---------- 삼각형(simplex = {A, B, C}) 처리 ----------
            // A: 가장 최근 추가된 점
            Vector3 A = simplex[2];
            Vector3 B = simplex[1];
            Vector3 C = simplex[0];
            Vector3 AB = B - A;
            Vector3 AC = C - A;
            Vector3 AO = -A;

            // 삼각형의 평면의 법선 (정규화 전)
            Vector3 ABC = AB.Cross(AC);

            // Edge AB에 대한 외측 법선 (삼각형의 평면 내에서, 원점 방향을 향함)
            Vector3 ABPerp = ABC.Cross(AB);
            if (ABPerp.Dot(AO) > 0) {
                // 원점이 edge AB 쪽에 있다면, C는 단순체에서 제거하고
                simplex.erase(simplex.begin()); // C 제거 (벡터 맨 앞)
                direction = AB.Cross(AO).Cross(AB);
                return false;
            }

            // Edge AC에 대한 외측 법선
            Vector3 ACPerp = AC.Cross(ABC);
            if (ACPerp.Dot(AO) > 0) {
                // 원점이 edge AC 쪽에 있다면, B는 단순체에서 제거합니다.
                simplex.erase(simplex.begin() + 1); // B 제거
                direction = AC.Cross(AO).Cross(AC);
                return false;
            }

            // 원점이 삼각형 내부에 있거나 삼각형 평면 상에 있으면, 검색 방향은 삼각형의 법선 쪽으로 설정합니다.
            if (ABC.Dot(AO) > 0) {
                direction = ABC;
            }
            else {
                // 방향 반전을 통해 올바른 방향을 취함
                direction = -ABC;
                // 단순체의 순서를 뒤바꿔서 동일한 결과를 얻도록 함
                std::swap(simplex[0], simplex[1]);
            }
        }
        else if (simplex.size() == 4) {
            // ---------- 사면체(simplex = {A, B, C, D}) 처리 ----------
            // A: 가장 최근 추가된 점
            Vector3 A = simplex[3];
            Vector3 B = simplex[2];
            Vector3 C = simplex[1];
            Vector3 D = simplex[0];
            Vector3 AO = -A;

            // 각 면의 법선 계산 (A를 기준으로 한 면)
            Vector3 ABC = (B - A).Cross(C - A);
            Vector3 ACD = (C - A).Cross(D - A);
            Vector3 ADB = (D - A).Cross(B - A);

            // 원점이 해당 면의 외측에 있으면, 단순체를 해당 면의 점들로 줄이고
            // 검색 방향을 그 면의 법선으로 설정합니다.
            if (ABC.Dot(AO) > 0) {
                // D는 기각합니다.
                simplex.erase(simplex.begin()); // D 제거
                direction = ABC;
                return false;
            }
            if (ACD.Dot(AO) > 0) {
                // B는 기각합니다.
                simplex.erase(simplex.begin() + 2); // B 제거
                direction = ACD;
                return false;
            }
            if (ADB.Dot(AO) > 0) {
                // C는 기각합니다.
                simplex.erase(simplex.begin() + 1); // C 제거
                direction = ADB;
                return false;
            }
            // 원점이 모든 면의 내부에 있다면, 두 객체는 충돌 중입니다.
            return true;
        }
        return false;
    }

} // namespace Collision