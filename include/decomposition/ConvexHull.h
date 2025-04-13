#ifndef CONVEXHULL_H
#define CONVEXHULL_H

class ConvexHull {
public:
    std::vector<Vector3> vertices;  // 정점
    std::vector<int> indices;       // 인덱스 (삼각형 면)
    
    // 특정 방향으로 가장 먼 점을 찾는 지원 함수 (GJK 알고리즘용)
    Vector3 getSupportPoint(const Vector3& direction) const;
};
#endif