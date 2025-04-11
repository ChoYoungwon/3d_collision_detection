#include "math/Vector3.h"
#include "geometry/AABB.h"
#include "geometry/OBB.h"
#include <vector>

class ConvexHull {
    public:
        std::vector<Math::Vector3> vertices;
        std::vector<int> indices;  // 삼각형 인덱스
        
        // ConvexHull 생성 및 변환
        static ConvexHull fromMesh(const std::vector<Math::Vector3>& meshVertices, const std::vector<int>& meshIndices);
        OBB computeOBB() const;
        AABB computeAABB() const;
        
        // Support 함수 - GJK에서 사용
        Math::Vector3 support(const Math::Vector3& direction) const;
    };