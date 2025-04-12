// src/collision/BVH.cpp
#include "collision/BVH.h"
#include <algorithm>
#include <limits>

namespace Collision {

    // BVHNode 구현
    BVHNode::BVHNode() : object(nullptr) {}

    BVHNode::~BVHNode() {}

    bool BVHNode::isLeaf() const {
        return !left && !right;
    }

    // BVH 구현
    BVH::BVH() : m_root(nullptr) {}

    BVH::~BVH() {}

    void BVH::build(const std::vector<Core::Object3D*>& objects) {
        if (objects.empty()) {
            m_root = nullptr;
            return;
        }

        m_objects = objects;

        // 각 객체의 바운딩 볼륨 업데이트
        for (auto obj : m_objects) {
            obj->updateBoundingVolumes();
        }

        // BVH 트리 구축
        m_root = buildRecursive(m_objects, 0, m_objects.size());
    }

    std::unique_ptr<BVHNode> BVH::buildRecursive(std::vector<Core::Object3D*>& objects, int start, int end) {
        auto node = std::make_unique<BVHNode>();

        // 리프 노드: 단일 객체
        if (end - start == 1) {
            node->object = objects[start];
            node->aabb = objects[start]->getAABB();
            return node;
        }

        // 현재 노드에 포함된 모든 객체의 AABB 계산
        Geometry::AABB totalAABB;
        for (int i = start; i < end; i++) {
            totalAABB.expand(objects[i]->getAABB());
        }
        node->aabb = totalAABB;

        // 가장 긴 축을 기준으로 정렬
        int axis = 0;
        float maxLength = totalAABB.getSize().x;
        if (totalAABB.getSize().y > maxLength) {
            axis = 1;
            maxLength = totalAABB.getSize().y;
        }
        if (totalAABB.getSize().z > maxLength) {
            axis = 2;
        }

        // 선택된 축을 기준으로 객체 정렬
        std::sort(objects.begin() + start, objects.begin() + end,
            [axis](Core::Object3D* a, Core::Object3D* b) {
                float centroidA = a->getAABB().getCenter()[axis];
                float centroidB = b->getAABB().getCenter()[axis];
                return centroidA < centroidB;
            });

        // 중간 지점으로 분할
        int mid = start + (end - start) / 2;

        // 재귀적으로 자식 노드 구축
        node->left = buildRecursive(objects, start, mid);
        node->right = buildRecursive(objects, mid, end);

        return node;
    }

    void BVH::update() {
        // 모든 객체의 바운딩 볼륨 업데이트
        for (auto obj : m_objects) {
            obj->updateBoundingVolumes();
        }

        // 단순 구현: 트리 재구축
        // 최적화: 바운딩 볼륨만 업데이트하는 방식으로 개선 가능
        build(m_objects);
    }

    std::vector<std::pair<Core::Object3D*, Core::Object3D*>> BVH::findPotentialCollisions() {
        std::vector<std::pair<Core::Object3D*, Core::Object3D*>> collisions;

        if (!m_root) {
            return collisions;
        }

        // 재귀적으로 충돌 쌍 검출
        findCollisionsRecursive(m_root.get(), m_root.get(), collisions);

        return collisions;
    }

    void BVH::findCollisionsRecursive(BVHNode* node1, BVHNode* node2,
        std::vector<std::pair<Core::Object3D*, Core::Object3D*>>& collisions) {
        // 노드의 AABB가 겹치지 않으면 종료
        if (!node1->aabb.intersects(node2->aabb)) {
            return;
        }

        // 두 노드가 모두 리프 노드인 경우
        if (node1->isLeaf() && node2->isLeaf()) {
            // 같은 객체는 제외
            if (node1->object != node2->object) {
                collisions.push_back(std::make_pair(node1->object, node2->object));
            }
            return;
        }

        // node1이 리프이고 node2가 내부 노드인 경우
        if (node1->isLeaf()) {
            findCollisionsRecursive(node1, node2->left.get(), collisions);
            findCollisionsRecursive(node1, node2->right.get(), collisions);
            return;
        }

        // node2가 리프이고 node1이 내부 노드인 경우
        if (node2->isLeaf()) {
            findCollisionsRecursive(node1->left.get(), node2, collisions);
            findCollisionsRecursive(node1->right.get(), node2, collisions);
            return;
        }

        // 두 노드가 모두 내부 노드인 경우
        findCollisionsRecursive(node1->left.get(), node2->left.get(), collisions);
        findCollisionsRecursive(node1->left.get(), node2->right.get(), collisions);
        findCollisionsRecursive(node1->right.get(), node2->left.get(), collisions);
        findCollisionsRecursive(node1->right.get(), node2->right.get(), collisions);
    }

    std::vector<Core::Object3D*> BVH::findPotentialCollisions(Core::Object3D* object) {
        std::vector<Core::Object3D*> collisions;

        if (!m_root || !object) {
            return collisions;
        }

        // 임시 BVH 노드 생성
        BVHNode tempNode;
        tempNode.object = object;
        tempNode.aabb = object->getAABB();

        // 재귀적으로 충돌 객체 검출 (다른 버전의 재귀 함수 필요)
        // 여기서는 간단한 구현만 제공
        for (auto obj : m_objects) {
            if (obj != object && obj->getAABB().intersects(object->getAABB())) {
                collisions.push_back(obj);
            }
        }

        return collisions;
    }

} // namespace Collision