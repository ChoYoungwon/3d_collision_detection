#include "collision/BVH.h"
#include <limits>

namespace Collision {

void BVH::rebuild() {
    if (objects.empty()) {
        root = nullptr;
        return;
    }

    // 모든 객체의 AABB를 최신 상태로 업데이트 (객체 내부에서 isDirty 플래그에 따라)
    for (auto obj : objects) {
        if (obj->isDirty) {
            obj->updateBoundingVolumes();
        }
    }

    // 객체 목록의 복사본을 사용하여 루트 노드를 재구성
    std::vector<Core::Object3D*> objectsCopy = objects;
    root = buildNode(objectsCopy, 0);
}

std::shared_ptr<BVHNode> BVH::buildNode(std::vector<Core::Object3D*>& nodeObjects, int depth) {
    auto node = std::make_shared<BVHNode>();

    // 해당 노드의 AABB: 포함하는 모든 객체들의 AABB를 합친 것
    node->aabb = AABB();
    for (auto obj : nodeObjects) {
        node->aabb.expand(obj->getAABB());
    }

    // 종료 조건: 객체가 1개 또는 최대 재귀 깊이에 도달하면 리프 노드로 처리
    const int MAX_DEPTH = 20;
    if (nodeObjects.size() == 1 || depth >= MAX_DEPTH) {
        if (!nodeObjects.empty()) {
            node->object = nodeObjects[0];
        }
        return node;
    }

    // 분할 축 선택: AABB 크기에서 가장 긴 축을 선택 (x:0, y:1, z:2)
    Vector3 size = node->aabb.getSize();
    int axis = 0;
    if (size.y > size.x && size.y > size.z) {
        axis = 1;
    } else if (size.z > size.x && size.z > size.y) {
        axis = 2;
    }

    // 선택한 축에 따라 분할할 위치(중심)
    Vector3 center = node->aabb.getCenter();
    float splitPos = (axis == 0) ? center.x : ((axis == 1) ? center.y : center.z);

    // 객체들을 좌우로 분할
    std::vector<Core::Object3D*> leftObjects, rightObjects;
    for (auto obj : nodeObjects) {
        Vector3 objCenter = obj->getAABB().getCenter();
        float objPos = (axis == 0) ? objCenter.x : ((axis == 1) ? objCenter.y : objCenter.z);
        if (objPos < splitPos) {
            leftObjects.push_back(obj);
        } else {
            rightObjects.push_back(obj);
        }
    }

    // 한쪽에만 몰리는 경우 단순 분할 (중간값 기준)
    if (leftObjects.empty() || rightObjects.empty()) {
        size_t mid = nodeObjects.size() / 2;
        leftObjects.clear();
        rightObjects.clear();
        for (size_t i = 0; i < nodeObjects.size(); i++) {
            if (i < mid)
                leftObjects.push_back(nodeObjects[i]);
            else
                rightObjects.push_back(nodeObjects[i]);
        }
    }

    // 재귀적으로 왼쪽, 오른쪽 자식 노드 생성
    node->left = buildNode(leftObjects, depth + 1);
    node->right = buildNode(rightObjects, depth + 1);

    return node;
}

void BVH::findCollisions(std::shared_ptr<BVHNode> nodeA, std::shared_ptr<BVHNode> nodeB,
    std::vector<std::pair<Core::Object3D*, Core::Object3D*>>& collisionPairs)
{
    // 두 노드의 AABB가 교차하지 않으면 바로 리턴
    if (!nodeA->aabb.intersects(nodeB->aabb))
        return;

    // 두 노드 모두 리프인 경우 충돌 가능한 객체 쌍 추가 (자기 자신 제외)
    if (nodeA->isLeaf() && nodeB->isLeaf()) {
        if (nodeA->object == nodeB->object)
            return;
        collisionPairs.push_back(std::make_pair(nodeA->object, nodeB->object));
        return;
    }

    // 한쪽이 리프이면 자식 노드와 비교
    if (nodeA->isLeaf()) {
        findCollisions(nodeA, nodeB->left, collisionPairs);
        findCollisions(nodeA, nodeB->right, collisionPairs);
        return;
    }
    if (nodeB->isLeaf()) {
        findCollisions(nodeA->left, nodeB, collisionPairs);
        findCollisions(nodeA->right, nodeB, collisionPairs);
        return;
    }

    // 두 노드 모두 내부 노드이면 자식끼리 4분할하여 비교
    findCollisions(nodeA->left, nodeB->left, collisionPairs);
    findCollisions(nodeA->left, nodeB->right, collisionPairs);
    findCollisions(nodeA->right, nodeB->left, collisionPairs);
    findCollisions(nodeA->right, nodeB->right, collisionPairs);
}

std::vector<std::pair<Core::Object3D*, Core::Object3D*>> BVH::findCollisionPairs() {
    std::vector<std::pair<Core::Object3D*, Core::Object3D*>> collisionPairs;
    if (!root || objects.size() < 2)
        return collisionPairs;

    // 객체들의 AABB를 다시 업데이트 (필요한 경우)
    for (auto obj : objects) {
        if (obj->isDirty) {
            obj->updateBoundingVolumes();
        }
    }

    findCollisions(root, root, collisionPairs);
    return collisionPairs;
}

} // namespace Collision
