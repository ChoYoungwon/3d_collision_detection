#include "GJK.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <set>
#include <tuple>
#include <functional>
#include "Quaternion.h"

namespace Collision {

    // 위치 정보를 포함한 새 Support 함수
    Vector3 GJK::Support(const ConvexHull& shapeA, const ConvexHull& shapeB, 
                        const Vector3& dir,
                        const Vector3& posA, const Vector3& posB) {
        // 로컬 좌표에서 지원점 계산
        Vector3 localPointA = shapeA.support(dir);
        Vector3 localPointB = shapeB.support(-dir);
        
        // 월드 좌표로 변환
        Vector3 worldPointA = localPointA + posA;
        Vector3 worldPointB = localPointB + posB;
        
        // Minkowski 차 계산
        return worldPointA - worldPointB;
    }

    // ConvexHull에서 특정 방향으로 가장 멀리 있는 점 찾기
    Vector3 GJK::getFarthestPointInDirection(const ConvexHull& shape, 
        const Vector3& dir, 
        const Vector3& position
    ) {
        if (shape.vertices.empty()) {
            return position;
    }

        Vector3 furthestPoint;
        float maxDot = -std::numeric_limits<float>::max();

        for (const auto& vertex : shape.vertices) {
            // 정점을 월드 좌표로 변환
            Vector3 worldVertex = vertex + position;
            float dot = worldVertex.dot(dir);

            if (dot > maxDot) {
                maxDot = dot;
                furthestPoint = worldVertex;
        }
    }

        return furthestPoint;
    }

    bool GJK::Intersect(
        const ConvexHull& shapeA, 
        const ConvexHull& shapeB,
        const Vector3& posA, 
        const Vector3& posB
    ) {
        // 초기 방향: B에서 A 방향 (보통 더 잘 수렴)
        Vector3 direction = (posA - posB).normalized();
        if (direction.magnitudeSquared() < 1e-6f) {
        direction = Vector3(1, 0, 0); // 방향이 너무 작으면 기본값 사용
        }

        std::vector<Vector3> simplex;

        // 빈 ConvexHull 체크
        if (shapeA.vertices.empty() || shapeB.vertices.empty()) {
            return false;
        }

        // 초기 지원점 계산 - 객체 위치 전달
        Vector3 initialPoint = Support(shapeA, shapeB, direction, posA, posB);
        simplex.push_back(initialPoint);

        // 새로운 검색 방향: 원점 방향
        direction = -initialPoint; // 초기 점에서 원점 방향

        // 방향 벡터 정규화
        float dirMag = direction.magnitude();
        if (dirMag < 1e-10f) {
            return true; // 초기 점이 원점과 일치하면 충돌 중
        }
        direction = direction / dirMag;

        // 최대 반복 횟수 및 수치 오차 허용값 설정
        const int MAX_ITERATIONS = 64; // 반복 횟수 증가
        const float EPSILON = 1e-10f;  // 정밀도 향상

        int iterationCount = 0;
        while (iterationCount < MAX_ITERATIONS) {
            iterationCount++;

            // 새 지원점 계산 - 객체 위치 전달
            Vector3 newPoint = Support(shapeA, shapeB, direction, posA, posB);

            // 새 점이 원점을 지나지 못하면 충돌 없음
            float dotProduct = newPoint.dot(direction);
            if (dotProduct < -EPSILON) { // 음수 임계값으로 수정
                return false;
            }

            // 새 점 추가
            simplex.push_back(newPoint);

            // 심플렉스가 원점을 포함하는지 확인하고 새 방향 계산
            if (DoSimplex(simplex, direction)) {
                return true; // 원점을 포함하므로 충돌
            }

            // 방향 벡터 정규화
            dirMag = direction.magnitude();
            if (dirMag < EPSILON) {
                return false; // 방향 벡터가 너무 작으면 이전에 검사한 것과 동일
            }
            direction = direction / dirMag;
        }

        // 최대 반복 횟수에 도달하면 결과 결정
        // 더 견고한 구현을 위해 심플렉스와 원점 간 거리 계산
        return calculateDistanceToOrigin(simplex) < EPSILON;
    }


    // 원점과 심플렉스 간 거리 계산
    float GJK::calculateDistanceToOrigin(const std::vector<Vector3>& simplex) {
        // 단순 구현: 심플렉스의 모든 점에서 원점까지의 최소 거리
        float minDistSq = std::numeric_limits<float>::max();
        
        for (const auto& point : simplex) {
            float distSq = point.magnitudeSquared();
            if (distSq < minDistSq) {
                minDistSq = distSq;
            }
        }
        
        return std::sqrt(minDistSq);
    }


    bool GJK::DoSimplex(std::vector<Vector3>& simplex, Vector3& direction) {
        switch (simplex.size()) {
            case 2: return doLine(simplex, direction);
            case 3: return doTriangle(simplex, direction);
            case 4: return doTetrahedron(simplex, direction);
            default: return false;
        }
    }
    
    // 선분 처리 (2점 심플렉스)
    bool GJK::doLine(std::vector<Vector3>& simplex, Vector3& direction) {
        Vector3 A = simplex[1]; // 마지막에 추가된 점
        Vector3 B = simplex[0]; // 이전 점
        Vector3 AB = B - A;     // B에서 A로의 벡터
        Vector3 AO = -A;        // A에서 원점으로의 벡터
        
        // AB 방향의 보로노이 영역에 원점이 있는지 확인
        if (AB.dot(AO) > 0) {
            // AB 방향으로 진행하되, AB⊥AO 방향으로 검색
            direction = AB.cross(AO).cross(AB);
            
            // 방향 벡터가 너무 작으면 임의의 수직 벡터 생성
            if (direction.magnitudeSquared() < 1e-6f) {
                // AB에 수직인 방향 찾기
                if (std::abs(AB.x) > std::abs(AB.y))
                    direction = Vector3(-AB.z, 0, AB.x);
                else
                    direction = Vector3(0, -AB.z, AB.y);
            }
        } else {
            // A를 유지하고 B 제거
            simplex.erase(simplex.begin()); // B 제거
            direction = AO; // 원점 방향
        }
        
        return false; // 원점이 아직 포함되지 않음
    }
    
    // 삼각형 처리 (3점 심플렉스)
    bool GJK::doTriangle(std::vector<Vector3>& simplex, Vector3& direction) {
        Vector3 A = simplex[2]; // 마지막 점
        Vector3 B = simplex[1]; 
        Vector3 C = simplex[0]; // 첫 점
        
        Vector3 AB = B - A;
        Vector3 AC = C - A;
        Vector3 AO = -A; // A에서 원점으로
        
        Vector3 ABCnormal = AB.cross(AC); // 삼각형 법선
        Vector3 ABperp = AB.cross(ABCnormal); // AB에 수직, ABC 평면 위
        Vector3 ACperp = ABCnormal.cross(AC); // AC에 수직, ABC 평면 위
        
        // AB 엣지의 보로노이 영역 확인
        if (ABperp.dot(AO) > 0) {
            // AB 엣지의 보로노이 영역에 원점이 있음
            simplex.erase(simplex.begin()); // C 제거
            return doLine(simplex, direction); // 선분으로 축소
        }
        
        // AC 엣지의 보로노이 영역 확인
        if (ACperp.dot(AO) > 0) {
            // AC 엣지의 보로노이 영역에 원점이 있음
            simplex.erase(simplex.begin() + 1); // B 제거
            return doLine(simplex, direction); // 선분으로 축소
        }
        
        // 삼각형 "위" 또는 "아래" 확인
        if (ABCnormal.dot(AO) > 0) {
            // 삼각형 "위"에 원점이 있음
            direction = ABCnormal;
        } else {
            // 삼각형 "아래"에 원점이 있음
            std::swap(simplex[0], simplex[1]); // B와 C 교체하여 일관된 방향 유지
            direction = -ABCnormal;
        }
        
        return false; // 아직 원점 포함 안됨
    }
    
    // 사면체 처리 (4점 심플렉스)
    bool GJK::doTetrahedron(std::vector<Vector3>& simplex, Vector3& direction) {
        Vector3 A = simplex[3]; // 마지막 점
        Vector3 B = simplex[2];
        Vector3 C = simplex[1];
        Vector3 D = simplex[0]; // 첫 점
        
        Vector3 AO = -A; // A에서 원점으로
        
        // 사면체의 각 면 확인
        Vector3 ABC = (B - A).cross(C - A);
        Vector3 ACD = (C - A).cross(D - A);
        Vector3 ADB = (D - A).cross(B - A);
        
        // 각 면의 바깥쪽을 향하는지 확인
        bool ABCout = ABC.dot(AO) > 0;
        bool ACDout = ACD.dot(AO) > 0;
        bool ADBout = ADB.dot(AO) > 0;
        
        // 모든 면에 대해 원점이 같은 쪽에 있는지 확인
        if (ABCout && ACDout && ADBout) {
            // 넷째 면 BCD도 확인
            Vector3 BDC = (D - B).cross(C - B);
            if (BDC.dot(B) > 0) { // B에서 원점 방향
                // 원점이 사면체 내부에 있음
                return true;
            }
        }
        
        // 원점이 어느 면의 바깥쪽에 있으면, 그 면을 향해 진행
        if (ABCout) {
            simplex.erase(simplex.begin()); // D 제거
            direction = ABC;
            return doTriangle(simplex, direction);
        }
        
        if (ACDout) {
            simplex.erase(simplex.begin() + 2); // B 제거
            direction = ACD;
            return doTriangle(simplex, direction);
        }
        
        if (ADBout) {
            simplex.erase(simplex.begin() + 1); // C 제거
            direction = ADB;
            return doTriangle(simplex, direction);
        }
        
        // 여기까지 왔다면 원점이 사면체 내부에 있음
        return true;
    }

} // namespace Collision
