#include "CollisionManager.h"
#include <algorithm>
#include <limits>
#include <iostream>

// 생성자 
CollisionManager::CollisionManager()
    : broadPhaseAlgorithm(CollisionAlgorithm::AABB),
      narrowPhaseAlgorithm(CollisionAlgorithm::GJK),
      frameCount(0),
      collisionCheckInterval(1) {
}

// 충돌 감지를 수행할 3D 객체를 관리 목록에 추가
void CollisionManager::addObject(Object3D* object) {
    if (object && std::find(objects.begin(), objects.end(), object) == objects.end()) {
        objects.push_back(object);
    }
}

// 특정 객체를 관리 목록에서 제거, 관련 충돌 상태도 제거
void CollisionManager::removeObject(Object3D* object) {
    auto it = std::find(objects.begin(), objects.end(), object);
    if (it != objects.end()) {
        objects.erase(it);
        
        // 객체와 관련된 충돌 상태 제거
        for (auto it = collisionState.begin(); it != collisionState.end();) {
            if (it->first.first == object || it->first.second == object) {
                it = collisionState.erase(it);
            } else {
                ++it;
            }
        }
    }
}

// 모든 객체와 충돌 상태를 초기화
void CollisionManager::clearObjects() {
    objects.clear();
    collisionState.clear();
}

// 대락적 충돌 감지 알고리즘
void CollisionManager::setBroadPhaseAlgorithm(CollisionAlgorithm algorithm) {
    broadPhaseAlgorithm = algorithm;
}

// 정밀 충돌 감지 알고리즘
void CollisionManager::setNarrowPhaseAlgorithm(CollisionAlgorithm algorithm) {
    narrowPhaseAlgorithm = algorithm;
}

// 충돌 검사를 수행할 프레임 간격 설정
void CollisionManager::setCollisionCheckInterval(int interval) {
    collisionCheckInterval = std::max(1, interval);
}

// 충돌 감지 및 해결(메인 루프)
void CollisionManager::update() {
    std::cout << "CollisionManager::update() - 시작 (프레임 " << frameCount << ")" << std::endl;
    
    frameCount++;
    
    // 매 interval 프레임마다 충돌 검사 수행
    if (frameCount % collisionCheckInterval != 0) {
        std::cout << "  현재 프레임에서는 충돌 검사 건너뜀" << std::endl;
        return;
    }
    
    std::cout << "  객체 목록 크기: " << objects.size() << std::endl;
    
    // 1. 모든 객체의 AABB 업데이트
    std::cout << "  각 객체의 AABB 업데이트 중..." << std::endl;
    for (auto* obj : objects) {
        if (obj == nullptr) {
            std::cout << "    경고: 널 객체 발견!" << std::endl;
            continue;
        }
        std::cout << "    객체 '" << obj->getName() << "' 업데이트" << std::endl;
        obj->update();
    }
    
    // 2. 대략적 충돌 감지 단계 (Broad Phase)
    std::cout << "  대략적 충돌 감지(Broad Phase) 중..." << std::endl;
    std::vector<std::pair<Object3D*, Object3D*>> potentialCollisions;
    broadPhase(potentialCollisions);
    std::cout << "    잠재적 충돌 쌍: " << potentialCollisions.size() << "개" << std::endl;
    
    // 현재 충돌 중인 객체 쌍 추적을 위한 맵
    std::unordered_map<std::pair<Object3D*, Object3D*>, bool, ObjectPairHash> currentCollisions;
    
    // 3. 정밀 충돌 감지 단계 (Narrow Phase)
    std::cout << "  정밀 충돌 감지(Narrow Phase) 시작..." << std::endl;
    int pairCounter = 0;
    for (auto& pair : potentialCollisions) {
        pairCounter++;
        Object3D* objA = pair.first;
        Object3D* objB = pair.second;
        
        std::cout << "    쌍 " << pairCounter << ": '" << objA->getName() 
                  << "' 와 '" << objB->getName() << "'" << std::endl;
        
        // 정규화된 객체 쌍 (메모리 주소가 작은 객체가 먼저)
        std::pair<Object3D*, Object3D*> normalizedPair = 
            (objA < objB) ? std::make_pair(objA, objB) : std::make_pair(objB, objA);
        
        // 정밀 충돌 감지 수행
        CollisionInfo collisionInfo;
        std::cout << "      narrowPhase 실행..." << std::endl;
        bool isColliding = narrowPhase(objA, objB, collisionInfo);
        std::cout << "      충돌 결과: " << (isColliding ? "충돌함" : "충돌 없음") << std::endl;
        
        // 충돌 상태 업데이트
        currentCollisions[normalizedPair] = isColliding;
        
        if (isColliding) {
            std::cout << "      충돌 정보 객체에 추가" << std::endl;
            // 충돌 정보 객체에 추가
            objA->addCollision(collisionInfo);
            
            // 반대 방향 충돌 정보 생성
            CollisionInfo reverseInfo(
                objA, 
                collisionInfo.contactPoint,
                -collisionInfo.contactNormal,  // 법선 반대 방향
                collisionInfo.penetrationDepth
            );
            objB->addCollision(reverseInfo);
        }
        else {
            // 이전에 충돌 중이었으면 충돌 제거
            auto prevIt = collisionState.find(normalizedPair);
            if (prevIt != collisionState.end() && prevIt->second) {
                std::cout << "      이전 충돌 상태 제거" << std::endl;
                objA->removeCollision(objB);
                objB->removeCollision(objA);
            }
        }
    }
    
    // 이전에 충돌 중이었지만 이번 프레임에서 검사되지 않은 쌍 확인
    std::cout << "  이전 충돌 상태 정리 중..." << std::endl;
    int cleanupCounter = 0;
    for (const auto& pair : collisionState) {
        if (pair.second && currentCollisions.find(pair.first) == currentCollisions.end()) {
            cleanupCounter++;
            Object3D* objA = pair.first.first;
            Object3D* objB = pair.first.second;
            
            std::cout << "    이전 충돌 제거 " << cleanupCounter << ": '"
                      << objA->getName() << "' 와 '" << objB->getName() << "'" << std::endl;
            
            // 더 이상 충돌 중이 아니므로 제거
            objA->removeCollision(objB);
            objB->removeCollision(objA);
        }
    }
    
    // 충돌 상태 업데이트
    std::cout << "  충돌 상태 맵 업데이트" << std::endl;
    collisionState = std::move(currentCollisions);
    
    std::cout << "CollisionManager::update() - 완료" << std::endl;
}

// 대략적 충돌 감지 (Broad Phase)
void CollisionManager::broadPhase(std::vector<std::pair<Object3D*, Object3D*>>& potentialCollisions) {
    potentialCollisions.clear();
    
    // 모든 객체 쌍에 대해 AABB 충돌 검사
    for (size_t i = 0; i < objects.size(); ++i) {
        for (size_t j = i + 1; j < objects.size(); ++j) {
            Object3D* objA = objects[i];
            Object3D* objB = objects[j];
            
            // AABB 충돌 검사
            if (checkAABBCollision(objA, objB)) {
                potentialCollisions.emplace_back(objA, objB);
            }
        }
    }
}

// 정밀 충돌 감지 (Narrow Phase)
bool CollisionManager::narrowPhase(Object3D* objA, Object3D* objB, CollisionInfo& collisionInfo) {
    switch (narrowPhaseAlgorithm) {
        case CollisionAlgorithm::GJK:
            return checkGJKCollision(objA, objB, collisionInfo);
        case CollisionAlgorithm::SAT:
            return checkSATCollision(objA, objB, collisionInfo);
        case CollisionAlgorithm::AABB:
            // AABB는 충돌 정보를 제공하지 않으므로, 단순 충돌 여부만 반환
            return checkAABBCollision(objA, objB);
        case CollisionAlgorithm::CUSTOM:
            // 필요에 따라 사용자 정의 알고리즘 구현
            return false;
        default:
            return false;
    }
}

// 두 객체가 AABB가 교차하는지 확인
bool CollisionManager::checkAABBCollision(Object3D* objA, Object3D* objB) {
    const AABB& aabbA = objA->getAABB();
    const AABB& aabbB = objB->getAABB();
    
    // AABB 교차 테스트
    return (aabbA.min.x <= aabbB.max.x && aabbA.max.x >= aabbB.min.x) &&
           (aabbA.min.y <= aabbB.max.y && aabbA.max.y >= aabbB.min.y) &&
           (aabbA.min.z <= aabbB.max.z && aabbA.max.z >= aabbB.min.z);
}

// GJK 충돌 감지 (Gilbert-Johnson-Keerthi 알고리즘)
bool CollisionManager::checkGJKCollision(Object3D* objA, Object3D* objB, CollisionInfo& collisionInfo) {
    std::cout << ">> checkGJKCollision 시작: " << objA->getName() << " vs " << objB->getName() << std::endl;
    
    // 객체 위치 가져오기
    Vector3 posA = objA->getPosition();
    Vector3 posB = objB->getPosition();
    
    std::cout << "  객체 위치: " << objA->getName() << "=(" << posA.toString() << ")"
            << ", " << objB->getName() << "=(" << posB.toString() << ")" << std::endl;


    // objA와 objB가 볼록 분해되어 있는지 확인
    std::cout << "  객체 분해 상태: " << objA->getName() << "=" << (objA->isDecomposed() ? "분해됨" : "분해 안됨")
              << ", " << objB->getName() << "=" << (objB->isDecomposed() ? "분해됨" : "분해 안됨") << std::endl;
    
    if (objA->isDecomposed() && objB->isDecomposed()) {
        // 두 객체의 볼록 껍질들 간의 충돌 검사
        const std::vector<ConvexHull>& hullsA = objA->getConvexHulls();
        const std::vector<ConvexHull>& hullsB = objB->getConvexHulls();
        
        std::cout << "  볼록 껍질 개수: " << objA->getName() << "=" << hullsA.size() 
                  << ", " << objB->getName() << "=" << hullsB.size() << std::endl;
        
        int hullACounter = 0;
        for (const ConvexHull& hullA : hullsA) {
            hullACounter++;
            std::cout << "    " << objA->getName() << "의 껍질 #" << hullACounter 
                      << " (정점 수: " << hullA.vertices.size() << ")" << std::endl;
            
            int hullBCounter = 0;
            for (const ConvexHull& hullB : hullsB) {
                hullBCounter++;
                std::cout << "      " << objB->getName() << "의 껍질 #" << hullBCounter 
                          << " (정점 수: " << hullB.vertices.size() << ")" << std::endl;
                
                // GJK로 충돌 확인
                std::cout << "      GJK 충돌 검사 시작..." << std::endl;
                bool result = false;
                try {
                    result = gjkSolver.Intersect(hullA, hullB, posA, posB);
                    std::cout << "      GJK 결과: " << (result ? "충돌" : "충돌 없음") << std::endl;
                } catch (const std::exception& e) {
                    std::cout << "      GJK 예외 발생: " << e.what() << std::endl;
                } catch (...) {
                    std::cout << "      GJK에서 알 수 없는 예외 발생" << std::endl;
                }
                
                if (result) {
                    std::cout << "      충돌 감지됨! 충돌 정보 설정 중..." << std::endl;
                    // 충돌 정보 계산 - EPA 알고리즘을 구현해야 함
                    // 임시로 기본값 설정
                    collisionInfo.otherObject = objB;
                    collisionInfo.contactPoint = (objA->getPosition() + objB->getPosition()) * 0.5f;
                    collisionInfo.contactNormal = (objB->getPosition() - objA->getPosition()).normalized();
                    collisionInfo.penetrationDepth = 0.1f;  // 임시값
                    
                    std::cout << "  checkGJKCollision 완료: 충돌 감지" << std::endl;
                    return true;
                }
            }
        }
        std::cout << "  모든 볼록 껍질 쌍 검사 완료, 충돌 없음" << std::endl;
        return false;
    }
    else {
        std::cout << "  비분해 객체 처리: 단일 ConvexHull 생성" << std::endl;
        // 볼록 분해되지 않은 객체는 단일 ConvexHull로 처리
        ConvexHull hullA, hullB;
        
        // 임시 ConvexHull 생성
        hullA.vertices = objA->getVertices();
        hullB.vertices = objB->getVertices();
        
        std::cout << "  생성된 ConvexHull 정점 수: " << objA->getName() << "=" << hullA.vertices.size() 
                  << ", " << objB->getName() << "=" << hullB.vertices.size() << std::endl;
        
        // GJK로 충돌 확인
        std::cout << "  GJK 충돌 검사 시작..." << std::endl;
        bool result = false;
        try {
            result = gjkSolver.Intersect(hullA, hullB, posA, posB);
            std::cout << "  GJK 결과: " << (result ? "충돌" : "충돌 없음") << std::endl;
        } catch (const std::exception& e) {
            std::cout << "  GJK 예외 발생: " << e.what() << std::endl;
        } catch (...) {
            std::cout << "  GJK에서 알 수 없는 예외 발생" << std::endl;
        }
        
        if (result) {
            std::cout << "  충돌 감지됨! 충돌 정보 설정 중..." << std::endl;
            // 임시 충돌 정보
            collisionInfo.otherObject = objB;
            collisionInfo.contactPoint = (objA->getPosition() + objB->getPosition()) * 0.5f;
            collisionInfo.contactNormal = (objB->getPosition() - objA->getPosition()).normalized();
            collisionInfo.penetrationDepth = 0.1f;  // 임시값
            
            std::cout << "  checkGJKCollision 완료: 충돌 감지" << std::endl;
            return true;
        }
        
        std::cout << "  checkGJKCollision 완료: 충돌 없음" << std::endl;
        return false;
    }
}

// SAT 충돌 감지 (Separating Axis Theorem)
bool CollisionManager::checkSATCollision(Object3D* objA, Object3D* objB, CollisionInfo& collisionInfo) {
    // AABB를 가져옵니다
    const AABB& aabbA = objA->getAABB();
    const AABB& aabbB = objB->getAABB();
    
    // OBB 생성 - 먼저 AABB로부터 생성하고 나서 변환 적용
    OBB obbA(aabbA);
    obbA.center = objA->getPosition();
    obbA.orientation = objA->getTransformMatrix();
    
    OBB obbB(aabbB);
    obbB.center = objB->getPosition();
    obbB.orientation = objB->getTransformMatrix();
    
    // OBB의 intersects 메서드 사용
    if (obbA.intersects(obbB)) {
        // 충돌 정보 설정
        collisionInfo.otherObject = objB;
        collisionInfo.contactPoint = (objA->getPosition() + objB->getPosition()) * 0.5f;
        collisionInfo.contactNormal = (objB->getPosition() - objA->getPosition()).normalized();
        collisionInfo.penetrationDepth = 0.1f;  // 임시값
        
        return true;
    }
    
    return false;
}

// EPA 알고리즘 (Expanding Polytope Algorithm)
bool CollisionManager::epaCalculatePenetration(Object3D* objA, Object3D* objB, 
                                              const std::vector<Vector3>& polytope,
                                              CollisionInfo& collisionInfo) {
    // EPA 알고리즘 구현
    // 실제 구현에서는 polytope 확장 및 최소 침투 벡터 찾기 필요
    
    // 기본 더미 구현 - 실제 구현 필요
    return false;
}