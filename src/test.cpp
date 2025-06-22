#include <iostream>
#include <string>
#include <vector>
#include "Object3D.h"
#include "CollisionManager.h"
#include "DS_timer.h"
#include "DS_definitions.h"
#include <thread>
#include <chrono>

int main(int argc, char* argv[]) {
    if (argc > 2) {
        std::cout << "[usage]" << std::endl;
        std::cout << "./collision_test [object_num]" << std::endl;
        return -1;
    }

    int numObjects = (argc > 1) ? std::stoi(argv[1]) : 10; 
    std::cout << "설정 객체 수 : " << numObjects << std::endl;


    Object3D* templateObject = new Object3D("Object");
    // 2. OBJ 파일에서 메시 데이터 로드
    if (!templateObject->loadFromObjFile("bunny.obj")) {
        std::cerr << "Failed to load bunny.obj" << std::endl;
        delete templateObject;
        return 1;
    }
    
    // 3. (선택적) 볼록 분해 적용
    // 복잡한 모델의 경우 충돌 감지 성능 향상을 위해
    VHACDParameters params;
    params.maxConvexHulls = 128;  // 적절한 값으로 조정
    
    std::string inputFile = "bunny.obj";
    std::string outputFile = "bunny_decomposed.obj";
    
    if (templateObject->computeConvexDecomposition(inputFile, outputFile, params)) {
        std::cout << "Object1 decomposed successfully" << std::endl;
        templateObject->loadConvexDecomposition(outputFile);
    } else {
        std::cerr << "Failed to decompose template object" << std::endl;
        delete templateObject;
        return 1;
    }

    // 템플릿 객체의 볼록 껍질 가져오기
    const std::vector<ConvexHull>& templateHulls = templateObject->getConvexHulls();
    std::cout << "템플릿 객체의 볼록 껍질 개수: " << templateHulls.size() << std::endl;

    // 여러 객체 생성 및 템플릿 데이터 복사
    std::vector<Object3D*> objects;
    // 충돌 관리자 설정
    CollisionManager collisionManager;
    for (int i = 0; i < numObjects; i++) {
        Object3D* obj = new Object3D("Object" + std::to_string(i+1));
        obj -> copyMeshData(*templateObject);

        // 객체를 3D 공간에 무작위로 배치 (-20 ~ 20 사이)
        float x = static_cast<float>(rand() % 5);
        float y = static_cast<float>(rand() % 5);
        float z = static_cast<float>(rand() % 5);
        obj->setPosition(Vector3(x, y, z));

        obj->setOnCollisionEnter([obj](const CollisionInfo& info) {
            std::cout << obj->getName() << " collision enter with " << info.otherObject->getName() << std::endl;
            std::cout << "Contact point: " << info.contactPoint.toString() << std::endl;
            std::cout << "Contact normal: " << info.contactNormal.toString() << std::endl;
            std::cout << "Penetration depth: " << info.penetrationDepth << std::endl;
        });

        objects.push_back(obj);

        collisionManager.addObject(objects[i]);

        // 선택적으로 충돌 알고리즘 설정
        collisionManager.setNarrowPhaseAlgorithm(CollisionAlgorithm::GJK);  // GJK, SAT
    }

    delete templateObject;
    
    // 8. 시뮬레이션 루프
    bool running = true;
    int frame = 0;
    const int maxFrames = 5;  // 테스트 프레임 수

    DS_timer timer(maxFrames + 4);
    timer.setTimerName(0, (char*)"[전체 시뮬레이션 시간]");
    timer.setTimerName(1, (char*)"[충돌 감지 시간]");
    timer.setTimerName(2, (char*)"[AABB 탐색 시간]");
    timer.setTimerName(3, (char*)"[GJK 탐색 시간]");

    for(int i = 4; i < maxFrames + 4; i++) {
        std::string timerName = "[프레임" + std::to_string(i-4) + "]";
        timer.setTimerName(i, (char*)timerName.c_str());
    }
    
    timer.onTimer(0);
    std::cout << "Updating object positions..." << std::endl;
    while (running && frame < maxFrames) {
        timer.onTimer(frame + 4);
        std::cout << "\n--- Frame " << frame << " ---" << std::endl;
        
        for (int i = 0; i < numObjects; i++) {
        // 8.1. 객체 위치 업데이트 (서로 가까워지게)
            if (frame > 0) {
                Vector3 center(0,0,0);
                Vector3 pos = objects[i] -> getPosition();
                Vector3 direction = (center - pos).normalized();
                pos += direction * 1.0f;
                objects[i] -> setPosition(pos);

                std::cout << "Object" <<std::to_string(i) << " position: " << objects[i]->getPosition().toString() << std::endl;
            }
            
            // 8.2. 업데이트 및 충돌 감지
            std::cout << "Updating objects..." << std::endl;
            objects[i]->update();
        }

        std::cout << "Running collision detection..." << std::endl;
        timer.onTimer(1);
        collisionManager.update(&timer);
        timer.offTimer(1);
        
        timer.offTimer(frame + 4);
        frame++;
    }
    timer.offTimer(0);
    
    timer.printTimer();
    // 9. 리소스 정리
    for(int i = 0; i < numObjects; i++)
        delete objects[i];
    
    return 0;
}