#ifndef BVH_H
#define BVH_H

#include "../core/Object3D.h"      // Core::Object3D
#include "../geometry/AABB.h"       // Geometry::AABB
#include "../math/Vector3.h"        // Math::Vector3
#include <vector>
#include <memory>
#include <algorithm>

namespace Collision {

// BVH 노드 클래스
class BVHNode {
public:
    AABB aabb;                      // 노드가 감싸는 AABB
    std::shared_ptr<BVHNode> left;            // 왼쪽 자식
    std::shared_ptr<BVHNode> right;           // 오른쪽 자식
    Core::Object3D* object;                   // 리프 노드일 때 해당 객체 (없으면 nullptr)

    BVHNode() : object(nullptr) {}

    bool isLeaf() const {
        return !left && !right;
    }
};

// BVH 클래스 (충돌 탐색 구조)
class BVH {
private:
    std::shared_ptr<BVHNode> root;           // BVH 루트 노드
    std::vector<Core::Object3D*> objects;      // 관리하는 객체들

    // 재귀적으로 노드 생성 (start ~ end 범위의 객체들을 이용)
    std::shared_ptr<BVHNode> buildNode(std::vector<Core::Object3D*>& nodeObjects, int depth);

    // 두 노드 간 충돌 가능성이 있는 쌍을 찾음
    void findCollisions(std::shared_ptr<BVHNode> nodeA, std::shared_ptr<BVHNode> nodeB,
        std::vector<std::pair<Core::Object3D*, Core::Object3D*>>& collisionPairs);

public:
    BVH() : root(nullptr) {}

    // 객체 추가 시 BVH 재구축
    void addObject(Core::Object3D* object) {
        objects.push_back(object);
        rebuild();
    }

    // 객체 제거 시 BVH 재구축
    void removeObject(Core::Object3D* object) {
        auto it = std::find(objects.begin(), objects.end(), object);
        if (it != objects.end()) {
            objects.erase(it);
            rebuild();
        }
    }

    // 객체 업데이트 시(변경된 AABB 등을 반영) BVH 재구축
    void updateObject(Core::Object3D* object) {
        rebuild();
    }

    // BVH 재구축 (객체들의 AABB 업데이트 후)
    void rebuild();

    // 충돌 가능성 있는 객체 쌍들을 반환
    std::vector<std::pair<Core::Object3D*, Core::Object3D*>> findCollisionPairs();
};

} // namespace Collision

#endif // BVH_H
