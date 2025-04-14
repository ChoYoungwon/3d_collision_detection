#ifndef COLLISION_MANAGER_H
#define COLLISION_MANAGER_H

#include <vector>
#include <unordered_map>
#include <utility>
#include "Object3D.h"
#include "GJK.h"
#include "SAT.h"
#include "OBB.h"

// 충돌 감지 알고리즘 열거형
enum class CollisionAlgorithm {
    AABB,       // 축 정렬 경계 상자 충돌 감지
    GJK,        // Gilbert-Johnson-Keerthi 알고리즘
    SAT,        // Separating Axis Theorem
    CUSTOM      // 사용자 정의 알고리즘
};

// 충돌 쌍을 위한 해시 함수
struct ObjectPairHash {
    std::size_t operator()(const std::pair<Object3D*, Object3D*>& pair) const {
        return std::hash<void*>{}(static_cast<void*>(pair.first)) ^ 
               std::hash<void*>{}(static_cast<void*>(pair.second));
    }
};

// 충돌 감지와 해결을 관리하는 클래스
class CollisionManager {
private:
    std::vector<Object3D*> objects;                            // 충돌 감지 대상 객체들
    std::unordered_map<std::pair<Object3D*, Object3D*>, bool, ObjectPairHash> collisionState;  // 이전 충돌 상태

    CollisionAlgorithm broadPhaseAlgorithm;                    // 대략적 충돌 감지 알고리즘
    CollisionAlgorithm narrowPhaseAlgorithm;                   // 정밀 충돌 감지 알고리즘

    // 충돌 횟수 제한을 위한 프레임 카운터
    int frameCount;
    int collisionCheckInterval;

    // GJK 인스턴스
    Collision::GJK gjkSolver;

public:
    CollisionManager();
    ~CollisionManager() = default;

    // 객체 관리
    void addObject(Object3D* object);
    void removeObject(Object3D* object);
    void clearObjects();

    // 알고리즘 설정
    void setBroadPhaseAlgorithm(CollisionAlgorithm algorithm);
    void setNarrowPhaseAlgorithm(CollisionAlgorithm algorithm);
    void setCollisionCheckInterval(int interval);

    // 충돌 감지 및 해결
    void update();

private:
    // 충돌 감지 단계
    void broadPhase(std::vector<std::pair<Object3D*, Object3D*>>& potentialCollisions);
    bool narrowPhase(Object3D* objA, Object3D* objB, CollisionInfo& collisionInfo);

    // 특정 충돌 감지 알고리즘
    bool checkAABBCollision(Object3D* objA, Object3D* objB);
    bool checkGJKCollision(Object3D* objA, Object3D* objB, CollisionInfo& collisionInfo);
    bool checkSATCollision(Object3D* objA, Object3D* objB, CollisionInfo& collisionInfo);

    // GJK 알고리즘 관련 헬퍼 함수
    Vector3 getSupport(Object3D* objA, Object3D* objB, const Vector3& direction);
    bool gjkIntersection(Object3D* objA, Object3D* objB);
    
    // EPA 알고리즘 (충돌 정보 계산)
    bool epaCalculatePenetration(Object3D* objA, Object3D* objB, 
                                const std::vector<Vector3>& polytope,
                                CollisionInfo& collisionInfo);
};

#endif // COLLISION_MANAGER_H