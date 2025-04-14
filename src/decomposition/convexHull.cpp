#include "ConvexHull.h"
#include <cmath>
#include <algorithm>
#include <numeric>

// 볼록 껍질의 부피 계산 (근사치)
float ConvexHull::calculateVolume() const {
    // 부피 계산을 위해 사면체의 부피를 합산함
    // 각 삼각형과 원점(0,0,0)으로 사면체를 형성하여 부피 계산
    float volume = 0.0f;
    
    // 중심점 계산
    Vector3 centroid = calculateCentroid();
    
    for (size_t i = 0; i < indices.size(); i += 3) {
        // 삼각형의 세 정점
        const Vector3& v0 = vertices[indices[i]];
        const Vector3& v1 = vertices[indices[i + 1]];
        const Vector3& v2 = vertices[indices[i + 2]];
        
        // 중심점에서 각 정점으로의 벡터
        Vector3 a = v0 - centroid;
        Vector3 b = v1 - centroid;
        Vector3 c = v2 - centroid;
        
        // 사면체 부피 = |삼중 스칼라곱| / 6
        // 삼중 스칼라곱 = a · (b × c)
        Vector3 crossBC = b.cross(c);
        float tripleProduct = std::abs(a.dot(crossBC));
        
        volume += tripleProduct / 6.0f;
    }
    
    return volume;
}

// 볼록 껍질의 표면적 계산
float ConvexHull::calculateSurfaceArea() const {
    float area = 0.0f;
    
    for (size_t i = 0; i < indices.size(); i += 3) {
        // 삼각형의 세 정점
        const Vector3& v0 = vertices[indices[i]];
        const Vector3& v1 = vertices[indices[i + 1]];
        const Vector3& v2 = vertices[indices[i + 2]];
        
        // 두 변 벡터
        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;
        
        // 삼각형 면적 = |외적| / 2
        Vector3 cross = edge1.cross(edge2);
        area += cross.magnitude() / 2.0f;
    }
    
    return area;
}

// 볼록 껍질의 중심점 계산
Vector3 ConvexHull::calculateCentroid() const {
    if (vertices.empty()) {
        return Vector3(0.0f, 0.0f, 0.0f);
    }
    
    // 단순 평균으로 계산 (더 정확한 방법은 가중치를 고려한 계산)
    Vector3 sum(0.0f, 0.0f, 0.0f);
    
    for (const auto& vertex : vertices) {
        sum = sum + vertex;
    }
    
    return sum / static_cast<float>(vertices.size());
}

// 특정 방향에 최대로 멀리있는 점
Vector3 ConvexHull::support(const Vector3& direction) const {
    if (vertices.empty()) {
        return Vector3(0, 0, 0);
    }
    
    Vector3 furthestPoint = vertices[0];
    float maxDot = direction.dot(vertices[0]);
    
    for (size_t i = 1; i < vertices.size(); ++i) {
        float dot = direction.dot(vertices[i]);
        if (dot > maxDot) {
            maxDot = dot;
            furthestPoint = vertices[i];
        }
    }
    
    return furthestPoint;
}