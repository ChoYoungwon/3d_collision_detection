// include/collision/BVH.h
#ifndef BVH_H
#define BVH_H

#include "../core/Object3D.h"
#include "../geometry/AABB.h"
#include <vector>
#include <memory>
#include <algorithm>

// BVH ��� Ŭ����
class BVHNode {
public:
    AABB aabb;                      // ����� AABB
    std::shared_ptr<BVHNode> left;  // ���� �ڽ� ���
    std::shared_ptr<BVHNode> right; // ������ �ڽ� ���
    Object3D* object;               // ���� ��忡 ����� ��ü (���� ���� nullptr)

    // ������
    BVHNode() : object(nullptr) {}

    // ���� ������� Ȯ��
    bool isLeaf() const {
        return !left && !right;
    }
};

// BVH Ŭ����
class BVH {
private:
    std::shared_ptr<BVHNode> root;  // ��Ʈ ���
    std::vector<Object3D*> objects; // �����ϴ� ��ü��

public:
    // ������
    BVH() : root(nullptr) {}

    // ��ü �߰�
    void addObject(Object3D* object) {
        objects.push_back(object);
        rebuild(); // Ʈ�� �籸��
    }

    // ��ü ����
    void removeObject(Object3D* object) {
        auto it = std::find(objects.begin(), objects.end(), object);
        if (it != objects.end()) {
            objects.erase(it);
            rebuild(); // Ʈ�� �籸��
        }
    }

    // ��ü ������Ʈ (��ġ ���� ����� ���)
    void updateObject(Object3D* object) {
        // ������ Ʈ���� �籸��
        // ����ȭ: �κ������� �籸���ϴ� ����� ����
        rebuild();
    }

    // BVH Ʈ�� �籸��
    void rebuild() {
        if (objects.empty()) {
            root = nullptr;
            return;
        }

        // �� ��ü�� AABB ������Ʈ
        for (auto obj : objects) {
            if (obj->isDirty) {
                obj->updateBoundingVolumes();
            }
        }

        // ��ü ���纻���� �۾� (���� �������� ���)
        std::vector<Object3D*> objectsCopy = objects;

        // ��Ʈ ��� ���� �� ����� ����
        root = buildNode(objectsCopy, 0);
    }

    // �浹 �� ã��
    std::vector<std::pair<Object3D*, Object3D*>> findCollisionPairs() {
        std::vector<std::pair<Object3D*, Object3D*>> collisionPairs;

        if (!root || objects.size() < 2) {
            return collisionPairs;
        }

        // �� ��ü�� AABB ������Ʈ
        for (auto obj : objects) {
            if (obj->isDirty) {
                obj->updateBoundingVolumes();
            }
        }

        // ��������� �浹 �� ã��
        findCollisions(root, root, collisionPairs);

        return collisionPairs;
    }

private:
    // BVH ��� ����� ����
    std::shared_ptr<BVHNode> buildNode(std::vector<Object3D*>& nodeObjects, int depth) {
        std::shared_ptr<BVHNode> node = std::make_shared<BVHNode>();

        // ��� ��ü�� ���� AABB ���
        node->aabb = AABB();
        for (auto obj : nodeObjects) {
            node->aabb.expand(obj->aabb);
        }

        // ���� ����: ��ü�� �ϳ��̰ų� �ִ� ���̿� ����
        const int MAX_DEPTH = 20;
        if (nodeObjects.size() == 1 || depth >= MAX_DEPTH) {
            // ���� ��� ����
            if (!nodeObjects.empty()) {
                node->object = nodeObjects[0];
            }
            return node;
        }

        // ��ü���� ������ �� ���� (���� �� �� ���)
        Vector3 size = node->aabb.getSize();
        int axis = 0; // x�� �⺻��

        if (size.y > size.x && size.y > size.z) {
            axis = 1; // y��
        }
        else if (size.z > size.x && size.z > size.y) {
            axis = 2; // z��
        }

        // �߾Ӱ����� ����
        Vector3 center = node->aabb.getCenter();
        float splitPos = (axis == 0) ? center.x : ((axis == 1) ? center.y : center.z);

        // ��ü���� ���� �������� ����
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

        // ��� ��ü�� �������� �з��� ���, �߰������� ����
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

        // �ڽ� ��� ����� ����
        node->left = buildNode(leftObjects, depth + 1);
        node->right = buildNode(rightObjects, depth + 1);

        return node;
    }

    // ��������� �浹 �� ã��
    void findCollisions(std::shared_ptr<BVHNode> nodeA, std::shared_ptr<BVHNode> nodeB,
        std::vector<std::pair<Object3D*, Object3D*>>& collisionPairs) {
        // �� ����� AABB�� ��ġ�� ������ ����
        if (!nodeA->aabb.intersects(nodeB->aabb)) {
            return;
        }

        // �� ��尡 ��� ���� ����� ���
        if (nodeA->isLeaf() && nodeB->isLeaf()) {
            // ���� ��ü�� ��� ����
            if (nodeA->object == nodeB->object) {
                return;
            }

            // �浹 �� �߰�
            collisionPairs.push_back(std::make_pair(nodeA->object, nodeB->object));
            return;
        }

        // nodeA�� ���� ����̰� nodeB�� ���� ����� ���
        if (nodeA->isLeaf()) {
            findCollisions(nodeA, nodeB->left, collisionPairs);
            findCollisions(nodeA, nodeB->right, collisionPairs);
            return;
        }

        // nodeB�� ���� ����̰� nodeA�� ���� ����� ���
        if (nodeB->isLeaf()) {
            findCollisions(nodeA->left, nodeB, collisionPairs);
            findCollisions(nodeA->right, nodeB, collisionPairs);
            return;
        }

        // �� ��� ��� ���� ����� ���
        findCollisions(nodeA->left, nodeB->left, collisionPairs);
        findCollisions(nodeA->left, nodeB->right, collisionPairs);
        findCollisions(nodeA->right, nodeB->left, collisionPairs);
        findCollisions(nodeA->right, nodeB->right, collisionPairs);
    }
};

#endif // BVH_H