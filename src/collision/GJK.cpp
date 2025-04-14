#include "GJK.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <set>
#include <tuple>
#include <functional>

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
        
        // 빈 ConvexHull 체크
        if (shapeA.vertices.empty() || shapeB.vertices.empty()) {
            return false;
        }
        
        Vector3 initialPoint = Support(shapeA, shapeB, direction);
        simplex.push_back(initialPoint);
        
        // 새로운 검색 방향: 원점 반대 방향
        direction = -simplex[0];
        
        // 방향 벡터 정규화
        float dirMag = direction.magnitude();
        if (dirMag < 1e-10f) {
            return false;  // 방향 벡터가 너무 작으면 종료
        }
        direction = direction / dirMag;
    
        // 이전 지원점들의 중복 여부를 확인하기 위한 집합
        std::set<std::pair<int, int>> visitedPairs;
        std::set<std::tuple<int, int, int>> visitedTriples;
        
        int iterationCount = 0;
        const int MAX_ITERATIONS = 30;  // 더 작은 값으로 설정
        const float EPSILON = 1e-6f;    // 수치 오차 허용 값
        
        while (iterationCount < MAX_ITERATIONS) {
            iterationCount++;
            
            Vector3 newPoint = Support(shapeA, shapeB, direction);
            
            // 내적이 매우 작으면 원점이 객체 외부에 있음
            float dotProduct = newPoint.dot(direction);
            if (dotProduct < EPSILON) {
                return false;
            }
            
            // 새 점이 이미 심플렉스에 있는지 확인 (중복 지원점 감지)
            bool duplicate = false;
            for (const auto& existingPoint : simplex) {
                if ((newPoint - existingPoint).magnitudeSquared() < EPSILON) {
                    duplicate = true;
                    break;
                }
            }
            
            if (duplicate) {
                return false;  // 중복 지원점이 발견되면 종료
            }
            
            simplex.push_back(newPoint);
            
            // 심플렉스에 기반한 순환 패턴 감지
            if (simplex.size() == 2) {
                int hash1 = std::hash<float>{}(simplex[0].x + simplex[0].y + simplex[0].z);
                int hash2 = std::hash<float>{}(simplex[1].x + simplex[1].y + simplex[1].z);
                std::pair<int, int> pair = std::minmax(hash1, hash2);
                
                if (visitedPairs.find(pair) != visitedPairs.end()) {
                    return false;  // 동일한 쌍이 이미 방문됨
                }
                visitedPairs.insert(pair);
            }
            else if (simplex.size() == 3) {
                int hash1 = std::hash<float>{}(simplex[0].x + simplex[0].y + simplex[0].z);
                int hash2 = std::hash<float>{}(simplex[1].x + simplex[1].y + simplex[1].z);
                int hash3 = std::hash<float>{}(simplex[2].x + simplex[2].y + simplex[2].z);
                std::tuple<int, int, int> triple = std::make_tuple(
                    std::min({hash1, hash2, hash3}),
                    std::max({hash1, hash2, hash3}),
                    hash1 + hash2 + hash3 - std::min({hash1, hash2, hash3}) - std::max({hash1, hash2, hash3})
                );
                
                if (visitedTriples.find(triple) != visitedTriples.end()) {
                    return false;  // 동일한 삼중체가 이미 방문됨
                }
                visitedTriples.insert(triple);
            }
            
            if (HandleSimplex(simplex, direction)) {
                return true;  // 원점이 심플렉스 내부에 있음
            }
            
            // 방향 벡터 정규화
            dirMag = direction.magnitude();
            if (dirMag < EPSILON) {
                return false;  // 방향 벡터가 너무 작으면 종료
            }
            direction = direction / dirMag;
        }
        
        return false;  // 최대 반복 횟수 도달
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
            // 모든 면에 대해 원점이 내부에 있다면 두 물체는 충돌 중
            return true;
        }
        return false;
    }

} // namespace Collision
