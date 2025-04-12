// include/core/CollisionManager.h
#ifndef COLLISION_MANAGER_H
#define COLLISION_MANAGER_H

#include "../collision/BVH.h"
#include "../core/Object3D.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <functional>

// �浹 ���� ����ü
struct CollisionInfo {
    Object3D* objectA;
    Object3D* objectB;
    bool hasCollision;
    // ���⿡ �浹 ����, �浹 ���� �� �߰� ������ Ȯ���� �� ����
};

// �浹 �ܰ� ������
enum class CollisionPhase {
    BROAD_PHASE,    // ���� �ܰ� (AABB)
    MID_PHASE,      // �߰� �ܰ� (OBB)
    NARROW_PHASE    // ���� �ܰ� (GJK ��)
};

// �浹 �ݹ� �Լ� Ÿ�� ����
using CollisionCallback = std::function<void(const CollisionInfo&)>;

class CollisionManager {
private:
    BVH bvh;                                                    // ���� �ܰ� BVH
    std::vector<Object3D*> objects;                             // �����ϴ� ��ü��
    std::unordered_map<std::string, CollisionCallback> callbacks; // �浹 �ݹ�
    bool useMultithreading;                                     // ��Ƽ������ ��� ����

public:
    // ������
    CollisionManager() : useMultithreading(false) {}

    // ��ü �߰�
    void addObject(Object3D* object) {
        objects.push_back(object);
        bvh.addObject(object);
    }

    // ��ü ����
    void removeObject(Object3D* object) {
        auto it = std::find(objects.begin(), objects.end(), object);
        if (it != objects.end()) {
            objects.erase(it);
            bvh.removeObject(object);
        }
    }

    // ��ü ������Ʈ
    void updateObject(Object3D* object) {
        if (object->isDirty) {
            object->updateBoundingVolumes();
            bvh.updateObject(object);
        }
    }

    // ��ü ��ü ������Ʈ
    void updateAllObjects() {
        for (auto obj : objects) {
            if (obj->isDirty) {
                obj->updateBoundingVolumes();
            }
        }
        bvh.rebuild();
    }

    // ���� �ܰ� �浹 ���� (AABB)
    std::vector<std::pair<Object3D*, Object3D*>> broadPhaseCollision() {
        return bvh.findCollisionPairs();
    }

    // �߰� �ܰ� �浹 ���� (OBB)
    std::vector<std::pair<Object3D*, Object3D*>> midPhaseCollision(
        const std::vector<std::pair<Object3D*, Object3D*>>& broadPhasePairs) {
        std::vector<std::pair<Object3D*, Object3D*>> midPhasePairs;

        // OpenMP ����ȭ (�ɼ�)
#pragma omp parallel for if(useMultithreading)
        for (size_t i = 0; i < broadPhasePairs.size(); i++) {
            auto& pair = broadPhasePairs[i];
            Object3D* objA = pair.first;
            Object3D* objB = pair.second;

            // OBB �浹 �˻�
            if (objA->intersectsOBB(*objB)) {
#pragma omp critical
                {
                    midPhasePairs.push_back(pair);
                }
            }
        }

        return midPhasePairs;
    }

    // ��ü �浹 ���� ���������� ����
    void detectCollisions() {
        // ��� ��ü ������Ʈ
        updateAllObjects();

        // 1. ���� �ܰ� (Broad Phase) - AABB ��� BVH
        std::vector<std::pair<Object3D*, Object3D*>> broadPhasePairs = broadPhaseCollision();

        // 2. �߰� �ܰ� (Mid Phase) - OBB
        std::vector<std::pair<Object3D*, Object3D*>> midPhasePairs = midPhaseCollision(broadPhasePairs);

        // 3. ���� �浹 ���� ���� �� �ݹ� ȣ��
        for (const auto& pair : midPhasePairs) {
            CollisionInfo info;
            info.objectA = pair.first;
            info.objectB = pair.second;
            info.hasCollision = true;

            // ��ϵ� ��� �ݹ� ȣ��
            for (const auto& cb : callbacks) {
                cb.second(info);
            }
        }
    }

    // �浹 �ݹ� ���
    void registerCollisionCallback(const std::string& name, CollisionCallback callback) {
        callbacks[name] = callback;
    }

    // �浹 �ݹ� ����
    void unregisterCollisionCallback(const std::string& name) {
        callbacks.erase(name);
    }

    // ��Ƽ������ ����
    void setMultithreading(bool enable) {
        useMultithreading = enable;
    }
};

#endif // COLLISION_MANAGER_H