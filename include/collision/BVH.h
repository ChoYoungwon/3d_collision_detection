// include/collision/BVH.h
#pragma once

#include <vector>
#include <memory>
#include "geometry/AABB.h"
#include "core/Object3D.h"

namespace Collision {

    // BVH 노드 클래스
    class BVHNode {
    public:
        BVHNode();
        ~BVHNode();

        // 자식 노드
        std::unique_ptr<BVHNode> left;
        std::unique_ptr<BVHNode> right;

        // 이 노드의 AABB
        Geometry::AABB aabb;

        // 리프 노드인 경우 해당 객체 참조
        Core::Object3D* object;

        // 리프 노드 여부
        bool isLeaf() const;
    };

    // BVH 트리 클래스
    class BVH {
    public:
        BVH();
        ~BVH();

        // BVH 트리 구축
        void build(const std::vector<Core::Object3D*>& objects);

        // BVH 트리 업데이트 (객체 이동 시)
        void update();

        // 충돌 가능성 있는 객체 쌍 검출
        std::vector<std::pair<Core::Object3D*, Core::Object3D*>> findPotentialCollisions();

        // 특정 객체와 충돌 가능성 있는 객체들 검출
        std::vector<Core::Object3D*> findPotentialCollisions(Core::Object3D* object);

    private:
        // BVH 트리의 루트 노드
        std::unique_ptr<BVHNode> m_root;

        // 트리에 포함된 객체들
        std::vector<Core::Object3D*> m_objects;

        // 재귀적으로 트리 구축 (내부 함수)
        std::unique_ptr<BVHNode> buildRecursive(std::vector<Core::Object3D*>& objects, int start, int end);

        // 재귀적으로 충돌 쌍 검출 (내부 함수)
        void findCollisionsRecursive(BVHNode* node1, BVHNode* node2,
            std::vector<std::pair<Core::Object3D*, Core::Object3D*>>& collisions);
    };

} // namespace Collision