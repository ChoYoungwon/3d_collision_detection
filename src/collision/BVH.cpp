// src/collision/BVH.cpp
#include "collision/BVH.h"
#include <algorithm>
#include <limits>

namespace Collision {

    // BVHNode ����
    BVHNode::BVHNode() : object(nullptr) {}

    BVHNode::~BVHNode() {}

    bool BVHNode::isLeaf() const {
        return !left && !right;
    }

    // BVH ����
    BVH::BVH() : m_root(nullptr) {}

    BVH::~BVH() {}

    void BVH::build(const std::vector<Core::Object3D*>& objects) {
        if (objects.empty()) {
            m_root = nullptr;
            return;
        }

        m_objects = objects;

        // �� ��ü�� �ٿ�� ���� ������Ʈ
        for (auto obj : m_objects) {
            obj->updateBoundingVolumes();
        }

        // BVH Ʈ�� ����
        m_root = buildRecursive(m_objects, 0, m_objects.size());
    }

    std::unique_ptr<BVHNode> BVH::buildRecursive(std::vector<Core::Object3D*>& objects, int start, int end) {
        auto node = std::make_unique<BVHNode>();

        // ���� ���: ���� ��ü
        if (end - start == 1) {
            node->object = objects[start];
            node->aabb = objects[start]->getAABB();
            return node;
        }

        // ���� ��忡 ���Ե� ��� ��ü�� AABB ���
        Geometry::AABB totalAABB;
        for (int i = start; i < end; i++) {
            totalAABB.expand(objects[i]->getAABB());
        }
        node->aabb = totalAABB;

        // ���� �� ���� �������� ����
        int axis = 0;
        float maxLength = totalAABB.getSize().x;
        if (totalAABB.getSize().y > maxLength) {
            axis = 1;
            maxLength = totalAABB.getSize().y;
        }
        if (totalAABB.getSize().z > maxLength) {
            axis = 2;
        }

        // ���õ� ���� �������� ��ü ����
        std::sort(objects.begin() + start, objects.begin() + end,
            [axis](Core::Object3D* a, Core::Object3D* b) {
                float centroidA = a->getAABB().getCenter()[axis];
                float centroidB = b->getAABB().getCenter()[axis];
                return centroidA < centroidB;
            });

        // �߰� �������� ����
        int mid = start + (end - start) / 2;

        // ��������� �ڽ� ��� ����
        node->left = buildRecursive(objects, start, mid);
        node->right = buildRecursive(objects, mid, end);

        return node;
    }

    void BVH::update() {
        // ��� ��ü�� �ٿ�� ���� ������Ʈ
        for (auto obj : m_objects) {
            obj->updateBoundingVolumes();
        }

        // �ܼ� ����: Ʈ�� �籸��
        // ����ȭ: �ٿ�� ������ ������Ʈ�ϴ� ������� ���� ����
        build(m_objects);
    }

    std::vector<std::pair<Core::Object3D*, Core::Object3D*>> BVH::findPotentialCollisions() {
        std::vector<std::pair<Core::Object3D*, Core::Object3D*>> collisions;

        if (!m_root) {
            return collisions;
        }

        // ��������� �浹 �� ����
        findCollisionsRecursive(m_root.get(), m_root.get(), collisions);

        return collisions;
    }

    void BVH::findCollisionsRecursive(BVHNode* node1, BVHNode* node2,
        std::vector<std::pair<Core::Object3D*, Core::Object3D*>>& collisions) {
        // ����� AABB�� ��ġ�� ������ ����
        if (!node1->aabb.intersects(node2->aabb)) {
            return;
        }

        // �� ��尡 ��� ���� ����� ���
        if (node1->isLeaf() && node2->isLeaf()) {
            // ���� ��ü�� ����
            if (node1->object != node2->object) {
                collisions.push_back(std::make_pair(node1->object, node2->object));
            }
            return;
        }

        // node1�� �����̰� node2�� ���� ����� ���
        if (node1->isLeaf()) {
            findCollisionsRecursive(node1, node2->left.get(), collisions);
            findCollisionsRecursive(node1, node2->right.get(), collisions);
            return;
        }

        // node2�� �����̰� node1�� ���� ����� ���
        if (node2->isLeaf()) {
            findCollisionsRecursive(node1->left.get(), node2, collisions);
            findCollisionsRecursive(node1->right.get(), node2, collisions);
            return;
        }

        // �� ��尡 ��� ���� ����� ���
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

        // �ӽ� BVH ��� ����
        BVHNode tempNode;
        tempNode.object = object;
        tempNode.aabb = object->getAABB();

        // ��������� �浹 ��ü ���� (�ٸ� ������ ��� �Լ� �ʿ�)
        // ���⼭�� ������ ������ ����
        for (auto obj : m_objects) {
            if (obj != object && obj->getAABB().intersects(object->getAABB())) {
                collisions.push_back(obj);
            }
        }

        return collisions;
    }

} // namespace Collision