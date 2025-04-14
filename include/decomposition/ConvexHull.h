#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include <vector>
#include "Vector3.h"

// 볼록 껍질 클래스 (단일 볼록 메시를 나타냄)
class ConvexHull {
public:
    std::vector<Vector3> vertices;  // 볼록 껍질의 정점 배열
    std::vector<int> indices;       // 면 인덱스 배열 (각 3개의 인덱스가 하나의 삼각형을 정의)
    
    // 기본 생성자
    ConvexHull() {}
    
    // 정점과 인덱스로 볼록 껍질 초기화
    ConvexHull(const std::vector<Vector3>& verts, const std::vector<int>& idx)
        : vertices(verts), indices(idx) {}
    
    // 정점 개수 반환
    size_t getVertexCount() const { return vertices.size(); }
    
    // 삼각형 개수 반환
    size_t getTriangleCount() const { return indices.size() / 3; }
    
    // 볼록 껍질의 부피 계산 (근사치)
    float calculateVolume() const;
    
    // 볼록 껍질의 표면적 계산
    float calculateSurfaceArea() const;
    
    // 볼록 껍질의 중심점 계산
    Vector3 calculateCentroid() const;

    // 특정 방향에 최대로 멀리있는 점
    Vector3 support(const Vector3& direction) const;
};

#endif // CONVEXHULL_H