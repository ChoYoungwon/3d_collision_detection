#include <iostream>
#include <string>
#include <vector>
#include "Object3D.h"
#include "CollisionManager.h"
#include <thread>
#include <chrono>

int main() {
    // 1. 두 개의 3D 객체 생성
    Object3D* object1 = new Object3D("Object1");
    Object3D* object2 = new Object3D("Object2");
    
    // 2. OBJ 파일에서 메시 데이터 로드
    if (!object1->loadFromObjFile("teddy.obj")) {
        std::cerr << "Failed to load model1.obj" << std::endl;
        delete object1;
        delete object2;
        return 1;
    }
    
    if (!object2->loadFromObjFile("cup.obj")) {
        std::cerr << "Failed to load model2.obj" << std::endl;
        delete object1;
        delete object2;
        return 1;
    }
    
    // 3. (선택적) 볼록 분해 적용
    // 복잡한 모델의 경우 충돌 감지 성능 향상을 위해
    VHACDParameters params;
    params.maxConvexHulls = 8;  // 적절한 값으로 조정
    
    std::string temp1 = "teddy.obj";
    std::string temp2 = "cup.obj";
    
    if (object1->computeConvexDecomposition(temp1, "teddy_decomposed.obj", params)) {
        std::cout << "Object1 decomposed successfully" << std::endl;
        object1->loadConvexDecomposition("model1_decomposed.obj");
    }
    
    if (object2->computeConvexDecomposition(temp2, "cup_decomposed.obj", params)) {
        std::cout << "Object2 decomposed successfully" << std::endl;
        object2->loadConvexDecomposition("model2_decomposed.obj");
    }
    
    // 4. 객체 위치 설정 (충돌 시나리오 생성)
    object1->setPosition(Vector3(0, 0, 0));
    object2->setPosition(Vector3(2, 0, 0));  // 처음에는 멀리 배치
    
    // 5. 충돌 콜백 설정
    object1->setOnCollisionEnter([](const CollisionInfo& info) {
        std::cout << "Object1 collision enter with " << info.otherObject->getName() << std::endl;
        std::cout << "Contact point: " << info.contactPoint.toString() << std::endl;
        std::cout << "Contact normal: " << info.contactNormal.toString() << std::endl;
        std::cout << "Penetration depth: " << info.penetrationDepth << std::endl;
    });
    
    object2->setOnCollisionEnter([](const CollisionInfo& info) {
        std::cout << "Object2 collision enter with " << info.otherObject->getName() << std::endl;
    });
    
    // 6. 충돌 관리자 설정
    CollisionManager collisionManager;
    collisionManager.addObject(object1);
    collisionManager.addObject(object2);
    
    // 7. 선택적으로 충돌 알고리즘 설정
    collisionManager.setNarrowPhaseAlgorithm(CollisionAlgorithm::GJK);  // GJK, SAT, AABB
    
    // 8. 시뮬레이션 루프
    bool running = true;
    int frame = 0;
    const int maxFrames = 100;  // 테스트 프레임 수
    
    while (running && frame < maxFrames) {
        std::cout << "\n--- Frame " << frame << " ---" << std::endl;
        
        // 디버그 출력 추가
        std::cout << "Updating object positions..." << std::endl;
        
        // 8.1. 객체 위치 업데이트 (서로 가까워지게)
        if (frame > 0) {
            Vector3 pos2 = object2->getPosition();
            pos2.x -= 0.05f;  // 각 프레임마다 x축으로 움직임
            object2->setPosition(pos2);
            
            std::cout << "Object1 position: " << object1->getPosition().toString() << std::endl;
            std::cout << "Object2 position: " << object2->getPosition().toString() << std::endl;
        }
        
        // 8.2. 업데이트 및 충돌 감지
        std::cout << "Updating objects..." << std::endl;
        object1->update();
        object2->update();

        std::cout << "Running collision detection..." << std::endl;
        collisionManager.update();
        
        // 8.3. 충돌 상태 출력
        std::cout << "Object1 colliding: " << (object1->isColliding() ? "Yes" : "No") << std::endl;
        std::cout << "Object2 colliding: " << (object2->isColliding() ? "Yes" : "No") << std::endl;
        
        frame++;

        // 잠시 대기 (프레임 관찰용)
        std::cout << "잠시 대기" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 9. 리소스 정리
    delete object1;
    delete object2;
    
    return 0;
}