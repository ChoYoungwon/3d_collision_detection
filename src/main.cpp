// main.cpp
#include <iostream>
#include <vector>
#include <string>

// GitHub 저장소 내 헤더 파일 (실제 경로와 이름은 프로젝트에 맞게 조정)
#include "geometry/ObjLoader.h"          // OBJ 파일 로드 기능 (분해된 모델 파일용)
#include "core/Object3D.h"               // 3D 객체의 기본 정보 (위치, 크기 등)
#include "collision/CollisionDetector.h" // 충돌 감지 기능 선언

int main() {
    // lamp.obj 파일 경로 (assets 폴더 내)
    std::string filePath = "assets/lamp_decomposed.obj";

    // 분해된 convex 파트들을 저장할 벡터
    std::vector<Object3D> lampParts;

    // OBJ 파일 로더 함수를 사용하여 lamp_decomposed.obj 파일 파싱
    if (!ObjLoader::LoadDecomposedObj(filePath, lampParts)) {
        std::cerr << "lamp.obj 파일 로드 실패: " << filePath << std::endl;
        return -1;
    }

    std::cout << "총 " << lampParts.size() << "개의 convex 파트를 로드하였습니다." << std::endl;

    // 예제: 로드된 파트 중 인덱스 0번과 1번 파트를 대상으로 충돌 검사
    if (lampParts.size() >= 2) {
        // 테스트를 위해 파트 1을 약간 이동시켜 충돌 조건을 변경할 수 있습니다.
        lampParts[1].translate({ 0.1f, 0.0f, 0.0f });

        bool collision = CollisionDetector::CheckCollision(lampParts[0], lampParts[1]);
        std::cout << "파트 0과 파트 1 충돌 여부: "
            << (collision ? "충돌 감지됨" : "충돌 없음") << std::endl;
    }
    else {
        std::cerr << "충돌 테스트를 위한 충분한 파트가 로드되지 않았습니다." << std::endl;
    }

    // 추가: 모든 파트 쌍에 대해 충돌 여부를 검사할 수 있습니다.
    for (size_t i = 0; i < lampParts.size(); ++i) {
        for (size_t j = i + 1; j < lampParts.size(); ++j) {
            bool collides = CollisionDetector::CheckCollision(lampParts[i], lampParts[j]);
            std::cout << "파트 " << i << "와 파트 " << j << " 충돌 여부: "
                << (collides ? "충돌" : "무충돌") << std::endl;
        }
    }

    return 0;
}