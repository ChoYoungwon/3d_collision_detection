// include/collision/BVH.h
#ifndef BVH_H
#define BVH_H

#include "../core/Object3D.h"
#include "../geometry/AABB.h"
#include <vector>
#include <memory>
#include <algorithm>

// BVH 노드 클래스
class BVHNode {
public:
    AABB aabb;                      // 노드의 AABB
    std::shared_ptr<BVHNode> left;  // 왼쪽 자식 노드
    std::shared_ptr<BVHNode> right; // 오른쪽 자식 노드
    Object3D* object;               // 리프 노드에 저장된 객체 (내부 노드는 nullptr)

    // 생성자
    BVHNode() : object(nullptr) {}

    // 리프 노드인지 확인
    bool isLeaf() const {
        return !left && !right;
    }
};

// BVH 클래스
class BVH {
private:
    std::shared_ptr<BVHNode> root;  // 루트 노드
    std::vector<Object3D*> objects; // 관리하는 객체들

public:
    // 생성자
    BVH() : root(nullptr) {}

    // 객체 추가
    void addObject(Object3D* object) {
        objects.push_back(object);
        rebuild(); // 트리 재구성
    }

    // 객체 제거
    void removeObject(Object3D* object) {
        auto it = std::find(objects.begin(), objects.end(), object);
        if (it != objects.end()) {
            objects.erase(it);
            rebuild(); // 트리 재구성
        }
    }

    // 객체 업데이트 (위치 등이 변경된 경우)
    void updateObject(Object3D* object) {
        // 간단히 트리를 재구성
        // 최적화: 부분적으로 재구성하는 방법도 있음
        rebuild();
    }

    // BVH 트리 재구성
    void rebuild() {
        if (objects.empty()) {
            root = nullptr;
            return;
        }

        // 각 객체의 AABB 업데이트
        for (auto obj : objects) {
            if (obj->isDirty) {
                obj->updateBoundingVolumes();
            }
        }

        // 객체 복사본으로 작업 (분할 과정에서 사용)
        std::vector<Object3D*> objectsCopy = objects;

        // 루트 노드 생성 및 재귀적 구성
        root = buildNode(objectsCopy, 0);
    }

    // 충돌 쌍 찾기
    std::vector<std::pair<Object3D*, Object3D*>> findCollisionPairs() {
        std::vector<std::pair<Object3D*, Object3D*>> collisionPairs;

        if (!root || objects.size() < 2) {
            return collisionPairs;
        }

        // 각 객체의 AABB 업데이트
        for (auto obj : objects) {
            if (obj->isDirty) {
                obj->updateBoundingVolumes();
            }
        }

        // 재귀적으로 충돌 쌍 찾기
        findCollisions(root, root, collisionPairs);

        return collisionPairs;
    }

private:
    // BVH 노드 재귀적 생성
    std::shared_ptr<BVHNode> buildNode(std::vector<Object3D*>& nodeObjects, int depth) {
        std::shared_ptr<BVHNode> node = std::make_shared<BVHNode>();

        // 모든 객체의 통합 AABB 계산
        node->aabb = AABB();
        for (auto obj : nodeObjects) {
            node->aabb.expand(obj->aabb);
        }

        // 종료 조건: 객체가 하나이거나 최대 깊이에 도달
        const int MAX_DEPTH = 20;
        if (nodeObjects.size() == 1 || depth >= MAX_DEPTH) {
            // 리프 노드 생성
            if (!nodeObjects.empty()) {
                node->object = nodeObjects[0];
            }
            return node;
        }

        // 객체들을 분할할 축 결정 (가장 긴 축 사용)
        Vector3 size = node->aabb.getSize();
        int axis = 0; // x축 기본값

        if (size.y > size.x && size.y > size.z) {
            axis = 1; // y축
        }
        else if (size.z > size.x && size.z > size.y) {
            axis = 2; // z축
        }

        // 중앙값으로 분할
        Vector3 center = node->aabb.getCenter();
        float splitPos = (axis == 0) ? center.x : ((axis == 1) ? center.y : center.z);

        // 객체들을 축을 기준으로 정렬
        std::vector<Object3D*> leftObjects, rightObjects;

        for (auto obj : nodeObjects) {
            Vector3 objCenter = obj->aabb.getCenter();
            float objPos = (axis == 0) ? objCenter.x : ((axis == 1) ? objCenter.y : objCenter.z);

            if (objPos < splitPos) {
                leftObjects.push_back(obj);
            }
            else {
                rightObjects.push_back(obj);
            }
        }

        // 모든 객체가 한쪽으로 분류된 경우, 중간점으로 분할
        if (leftObjects.empty() || rightObjects.empty()) {
            size_t mid = nodeObjects.size() / 2;
            leftObjects.clear();
            rightObjects.clear();

            for (size_t i = 0; i < nodeObjects.size(); i++) {
                if (i < mid) {
                    leftObjects.push_back(nodeObjects[i]);
                }
                else {
                    rightObjects.push_back(nodeObjects[i]);
                }
            }
        }

        // 자식 노드 재귀적 생성
        node->left = buildNode(leftObjects, depth + 1);
        node->right = buildNode(rightObjects, depth + 1);

        return node;
    }

    // 재귀적으로 충돌 쌍 찾기
    void findCollisions(std::shared_ptr<BVHNode> nodeA, std::shared_ptr<BVHNode> nodeB,
        std::vector<std::pair<Object3D*, Object3D*>>& collisionPairs) {
        // 두 노드의 AABB가 겹치지 않으면 종료
        if (!nodeA->aabb.intersects(nodeB->aabb)) {
            return;
        }

        // 두 노드가 모두 리프 노드인 경우
        if (nodeA->isLeaf() && nodeB->isLeaf()) {
            // 같은 객체인 경우 무시
            if (nodeA->object == nodeB->object) {
                return;
            }

            // 충돌 쌍 추가
            collisionPairs.push_back(std::make_pair(nodeA->object, nodeB->object));
            return;
        }

        // nodeA가 리프 노드이고 nodeB가 내부 노드인 경우
        if (nodeA->isLeaf()) {
            findCollisions(nodeA, nodeB->left, collisionPairs);
            findCollisions(nodeA, nodeB->right, collisionPairs);
            return;
        }

        // nodeB가 리프 노드이고 nodeA가 내부 노드인 경우
        if (nodeB->isLeaf()) {
            findCollisions(nodeA->left, nodeB, collisionPairs);
            findCollisions(nodeA->right, nodeB, collisionPairs);
            return;
        }

        // 두 노드 모두 내부 노드인 경우
        findCollisions(nodeA->left, nodeB->left, collisionPairs);
        findCollisions(nodeA->left, nodeB->right, collisionPairs);
        findCollisions(nodeA->right, nodeB->left, collisionPairs);
        findCollisions(nodeA->right, nodeB->right, collisionPairs);
    }
};

#endif // BVH_H