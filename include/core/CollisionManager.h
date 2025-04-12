// include/core/CollisionManager.h
#ifndef COLLISION_MANAGER_H
#define COLLISION_MANAGER_H

#include "../collision/BVH.h"
#include "../core/Object3D.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <functional>

// 충돌 정보 구조체
struct CollisionInfo {
    Object3D* objectA;
    Object3D* objectB;
    bool hasCollision;
    // 여기에 충돌 지점, 충돌 법선 등 추가 정보를 확장할 수 있음
};

// 충돌 단계 열거형
enum class CollisionPhase {
    BROAD_PHASE,    // 광역 단계 (AABB)
    MID_PHASE,      // 중간 단계 (OBB)
    NARROW_PHASE    // 협역 단계 (GJK 등)
};

// 충돌 콜백 함수 타입 정의
using CollisionCallback = std::function<void(const CollisionInfo&)>;

class CollisionManager {
private:
    BVH bvh;                                                    // 광역 단계 BVH
    std::vector<Object3D*> objects;                             // 관리하는 객체들
    std::unordered_map<std::string, CollisionCallback> callbacks; // 충돌 콜백
    bool useMultithreading;                                     // 멀티스레딩 사용 여부

public:
    // 생성자
    CollisionManager() : useMultithreading(false) {}

    // 객체 추가
    void addObject(Object3D* object) {
        objects.push_back(object);
        bvh.addObject(object);
    }

    // 객체 제거
    void removeObject(Object3D* object) {
        auto it = std::find(objects.begin(), objects.end(), object);
        if (it != objects.end()) {
            objects.erase(it);
            bvh.removeObject(object);
        }
    }

    // 객체 업데이트
    void updateObject(Object3D* object) {
        if (object->isDirty) {
            object->updateBoundingVolumes();
            bvh.updateObject(object);
        }
    }

    // 객체 전체 업데이트
    void updateAllObjects() {
        for (auto obj : objects) {
            if (obj->isDirty) {
                obj->updateBoundingVolumes();
            }
        }
        bvh.rebuild();
    }

    // 광역 단계 충돌 감지 (AABB)
    std::vector<std::pair<Object3D*, Object3D*>> broadPhaseCollision() {
        return bvh.findCollisionPairs();
    }

    // 중간 단계 충돌 감지 (OBB)
    std::vector<std::pair<Object3D*, Object3D*>> midPhaseCollision(
        const std::vector<std::pair<Object3D*, Object3D*>>& broadPhasePairs) {
        std::vector<std::pair<Object3D*, Object3D*>> midPhasePairs;

        // OpenMP 병렬화 (옵션)
#pragma omp parallel for if(useMultithreading)
        for (size_t i = 0; i < broadPhasePairs.size(); i++) {
            auto& pair = broadPhasePairs[i];
            Object3D* objA = pair.first;
            Object3D* objB = pair.second;

            // OBB 충돌 검사
            if (objA->intersectsOBB(*objB)) {
#pragma omp critical
                {
                    midPhasePairs.push_back(pair);
                }
            }
        }

        return midPhasePairs;
    }

    // 전체 충돌 감지 파이프라인 실행
    void detectCollisions() {
        // 모든 객체 업데이트
        updateAllObjects();

        // 1. 광역 단계 (Broad Phase) - AABB 기반 BVH
        std::vector<std::pair<Object3D*, Object3D*>> broadPhasePairs = broadPhaseCollision();

        // 2. 중간 단계 (Mid Phase) - OBB
        std::vector<std::pair<Object3D*, Object3D*>> midPhasePairs = midPhaseCollision(broadPhasePairs);

        // 3. 실제 충돌 정보 생성 및 콜백 호출
        for (const auto& pair : midPhasePairs) {
            CollisionInfo info;
            info.objectA = pair.first;
            info.objectB = pair.second;
            info.hasCollision = true;

            // 등록된 모든 콜백 호출
            for (const auto& cb : callbacks) {
                cb.second(info);
            }
        }
    }

    // 충돌 콜백 등록
    void registerCollisionCallback(const std::string& name, CollisionCallback callback) {
        callbacks[name] = callback;
    }

    // 충돌 콜백 제거
    void unregisterCollisionCallback(const std::string& name) {
        callbacks.erase(name);
    }

    // 멀티스레딩 설정
    void setMultithreading(bool enable) {
        useMultithreading = enable;
    }
};

#endif // COLLISION_MANAGER_H