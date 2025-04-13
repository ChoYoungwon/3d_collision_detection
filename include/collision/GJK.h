
#ifndef GJK_H
#define GJK_H

#include "../math/Vector3.h"
#include <algorithm>
#include <array>
#include <limits>
#include <vector>

// 충돌 객체의 인터페이스
class ConvexShape {
public:
    // 지원 함수 - 특정 방향에서 가장 먼 점을 반환
    virtual Vector3 getSupportPoint(const Vector3& direction) const = 0;
};


class GJK {
private:
    // 심플렉스 - 최대 4개의 점을 저장 (점, 선, 삼각형, 사면체)
    std::array<Vector3, 4> simplex;
    int simplexSize;

    // 민코프스키 차이의 지원 함수
    Vector3 getSupport(const ConvexShape& shapeA, const ConvexShape& shapeB, const Vector3& direction);

    // 원점이 심플렉스에 포함되어 있는지 확인하고 새로운 방향 결정
    bool processSimplex(Vector3& direction);

    bool processLine(Vector3& direction);

    bool processTriangle(Vector3& direction);

    bool processTetrahedron(Vector3& direction);

public:
    // 생성자
    GJK() : simplexSize(0) {}

    // 두 블록 도형 사이의 충돌 검사사
    bool checkCollision(const ConvexShape& shapeA, const ConvexShape& shapeB, int maxIterations = 32);

};

#endif 