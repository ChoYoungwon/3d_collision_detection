// include/collision/BVH.h
#pragma once

#include <vector>
#include <memory>
#include "geometry/AABB.h"
#include "core/Object3D.h"

namespace Collision {

    // BVH ��� Ŭ����
    class BVHNode {
    public:
        BVHNode();
        ~BVHNode();

        // �ڽ� ���
        std::unique_ptr<BVHNode> left;
        std::unique_ptr<BVHNode> right;

        // �� ����� AABB
        Geometry::AABB aabb;

        // ���� ����� ��� �ش� ��ü ����
        Core::Object3D* object;

        // ���� ��� ����
        bool isLeaf() const;
    };

    // BVH Ʈ�� Ŭ����
    class BVH {
    public:
        BVH();
        ~BVH();

        // BVH Ʈ�� ����
        void build(const std::vector<Core::Object3D*>& objects);

        // BVH Ʈ�� ������Ʈ (��ü �̵� ��)
        void update();

        // �浹 ���ɼ� �ִ� ��ü �� ����
        std::vector<std::pair<Core::Object3D*, Core::Object3D*>> findPotentialCollisions();

        // Ư�� ��ü�� �浹 ���ɼ� �ִ� ��ü�� ����
        std::vector<Core::Object3D*> findPotentialCollisions(Core::Object3D* object);

    private:
        // BVH Ʈ���� ��Ʈ ���
        std::unique_ptr<BVHNode> m_root;

        // Ʈ���� ���Ե� ��ü��
        std::vector<Core::Object3D*> m_objects;

        // ��������� Ʈ�� ���� (���� �Լ�)
        std::unique_ptr<BVHNode> buildRecursive(std::vector<Core::Object3D*>& objects, int start, int end);

        // ��������� �浹 �� ���� (���� �Լ�)
        void findCollisionsRecursive(BVHNode* node1, BVHNode* node2,
            std::vector<std::pair<Core::Object3D*, Core::Object3D*>>& collisions);
    };

} // namespace Collision