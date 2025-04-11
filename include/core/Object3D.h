#pragma once

#include <string>
#include <vector>
#include "math/Vector3.h"
#include "math/Quaternion.h"
#include "geometry/AABB.h"
#include "geometry/OBB.h"
#include "geometry/ConvexHull.h"
#include "decomposition/ConvexDecomposition.h"

class Object3D {
public:
    std::string name;
    std::vector<Math::Vector3> vertices;
    std::vector<int> indices;
    
    // 바운딩 볼륨
    AABB aabb;
    OBB obb;
    
    // V-HACD 분해 결과
    std::vector<ConvexHull> convexParts;
    
    // 변환
    Math::Vector3 position;
    Math::Quaternion rotation;
    Math::Vector3 scale;
    
    Object3D();
    Object3D(const std::string& name);
    Object3D(const std::vector<Math::Vector3>& vertices, const std::vector<int>& indices);
    
    // 메시 로드 및 처리
    bool loadFromFile(const std::string& filename);
    void computeBoundingVolumes();
    void decomposeToConvexHulls(const ConvexDecomposition::Parameters& params = ConvexDecomposition::Parameters());
    
    // 변환 메서드
    void setPosition(const Math::Vector3& pos);
    void setRotation(const Math::Quaternion& rot);
    void setScale(const Math::Vector3& s);
    Math::Matrix3x3 getTransformMatrix() const;
    
    // 객체 상태 갱신
    void update();
};