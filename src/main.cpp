#include <iostream>
#include <vector>
#include <chrono>
#include <omp.h>
#include "../include/math/Vector3.h"
#include "../include/math/Matrix3x3.h"
#include "../include/math/Quaternion.h"
#include "../include/geometry/AABB.h"
#include "../include/geometry/OBB.h"
#include "../include/core/Object3D.h"
#include "../include/collision/BVH.h"
#include "../include/collision/GJK.h"
#include "../include/collision/SAT.h"
#include "../include/core/CollisionManager.h"

namespace TeamMember1 {
    // 벡터 연산 테스트
    void testVectorOperations() {
        std::cout << "=== Vector3 연산 테스트 ===" << std::endl;

        // 벡터 생성 및 기본 연산
        Vector3 v1(1.0f, 2.0f, 3.0f);
        Vector3 v2(4.0f, 5.0f, 6.0f);

        std::cout << "v1: " << v1.toString() << std::endl;
        std::cout << "v2: " << v2.toString() << std::endl;

        // 벡터 덧셈
        Vector3 sum = v1 + v2;
        std::cout << "v1 + v2: " << sum.toString() << std::endl;

        // 벡터 뺄셈
        Vector3 diff = v2 - v1;
        std::cout << "v2 - v1: " << diff.toString() << std::endl;

        // 스칼라 곱
        Vector3 scaled = v1 * 2.0f;
        std::cout << "v1 * 2.0: " << scaled.toString() << std::endl;

        // 내적
        float dot = v1.dot(v2);
        std::cout << "v1 · v2: " << dot << std::endl;

        // 외적
        Vector3 cross = v1.cross(v2);
        std::cout << "v1 × v2: " << cross.toString() << std::endl;

        // 정규화
        Vector3 normalized = v1.normalized();
        std::cout << "v1 정규화: " << normalized.toString() << std::endl;
        std::cout << "정규화 벡터 길이: " << normalized.magnitude() << std::endl;
    }

    // 행렬 연산 테스트
    void testMatrixOperations() {
        std::cout << "\n=== Matrix3x3 연산 테스트 ===" << std::endl;

        // 단위 행렬 생성
        Matrix3x3 identity = Matrix3x3::identity();
        std::cout << "단위 행렬:\n" << identity.toString() << std::endl;

        // 회전 행렬 생성 (X축 90도 회전)
        Matrix3x3 rotX = Matrix3x3::rotationX(M_PI / 2);
        std::cout << "X축 90도 회전 행렬:\n" << rotX.toString() << std::endl;

        // 벡터 회전
        Vector3 v(1.0f, 0.0f, 0.0f);
        Vector3 rotated = rotX * v;
        std::cout << "X축 90도 회전 결과: " << rotated.toString() << std::endl;

        // 행렬 곱셈
        Matrix3x3 rotY = Matrix3x3::rotationY(M_PI / 4); // Y축 45도 회전
        Matrix3x3 combined = rotX * rotY;
        std::cout << "X축 90도 + Y축 45도 회전 행렬:\n" << combined.toString() << std::endl;

        // 행렬 전치
        Matrix3x3 transposed = combined.transpose();
        std::cout << "전치 행렬:\n" << transposed.toString() << std::endl;

        // 행렬 역행렬
        Matrix3x3 inverse = combined.inverse();
        std::cout << "역행렬:\n" << inverse.toString() << std::endl;

        // 역행렬 검증 (A * A^-1 = I)
        Matrix3x3 shouldBeIdentity = combined * inverse;
        std::cout << "A * A^-1 (단위 행렬이어야 함):\n" << shouldBeIdentity.toString() << std::endl;
    }

    // 쿼터니언 연산 테스트
    void testQuaternionOperations() {
        std::cout << "\n=== Quaternion 연산 테스트 ===" << std::endl;

        // 축-각 표현에서 쿼터니언 생성
        Vector3 axis(0.0f, 1.0f, 0.0f);
        float angle = M_PI / 2; // 90도
        Quaternion q1 = Quaternion::fromAxisAngle(axis, angle);
        std::cout << "Y축 90도 회전 쿼터니언: " << q1.toString() << std::endl;

        // 쿼터니언을 이용한 벡터 회전
        Vector3 v(1.0f, 0.0f, 0.0f);
        Vector3 rotated = q1.rotate(v);
        std::cout << "회전된 벡터: " << rotated.toString() << std::endl;

        // 쿼터니언 합성 (회전 순서: q1 다음 q2)
        Quaternion q2 = Quaternion::fromAxisAngle(Vector3(1.0f, 0.0f, 0.0f), M_PI / 4); // X축 45도
        Quaternion combined = q2 * q1;
        std::cout << "합성 쿼터니언 (Y축 90도 → X축 45도): " << combined.toString() << std::endl;

        // 합성 쿼터니언으로 벡터 회전
        Vector3 combinedRotated = combined.rotate(v);
        std::cout << "합성 회전된 벡터: " << combinedRotated.toString() << std::endl;

        // 쿼터니언 → 회전 행렬 변환
        Matrix3x3 rotMatrix = q1.toRotationMatrix();
        std::cout << "쿼터니언에서 변환된 회전 행렬:\n" << rotMatrix.toString() << std::endl;

        // 회전 행렬로 벡터 회전 (검증용)
        Vector3 rotatedByMatrix = rotMatrix * v;
        std::cout << "행렬로 회전된 벡터: " << rotatedByMatrix.toString() << std::endl;

        // 보간 테스트
        Quaternion q3 = Quaternion::identity();
        float t = 0.5f;
        Quaternion interpolated = Quaternion::slerp(q3, q1, t);
        std::cout << "q3와 q1 사이 50% 보간된 쿼터니언: " << interpolated.toString() << std::endl;
    }

    // AABB 충돌 테스트
    void testAABBCollision() {
        std::cout << "\n=== AABB 충돌 테스트 ===" << std::endl;

        // AABB 객체 생성
        AABB box1(Vector3(-1.0f, -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));
        AABB box2(Vector3(0.5f, 0.5f, 0.5f), Vector3(2.5f, 2.5f, 2.5f));
        AABB box3(Vector3(2.0f, 2.0f, 2.0f), Vector3(4.0f, 4.0f, 4.0f));

        std::cout << "Box1: " << box1.toString() << std::endl;
        std::cout << "Box2: " << box2.toString() << std::endl;
        std::cout << "Box3: " << box3.toString() << std::endl;

        // 충돌 검사
        bool collision12 = box1.intersects(box2);
        bool collision13 = box1.intersects(box3);
        bool collision23 = box2.intersects(box3);

        std::cout << "Box1과 Box2 충돌: " << (collision12 ? "예" : "아니오") << std::endl;
        std::cout << "Box1과 Box3 충돌: " << (collision13 ? "예" : "아니오") << std::endl;
        std::cout << "Box2과 Box3 충돌: " << (collision23 ? "예" : "아니오") << std::endl;

        // 합집합 AABB 생성
        AABB mergedBox = AABB::merge(box1, box2);
        std::cout << "Box1과 Box2의 합집합: " << mergedBox.toString() << std::endl;

        // 점 포함 여부 테스트
        Vector3 point1(0.0f, 0.0f, 0.0f);
        Vector3 point2(3.0f, 3.0f, 3.0f);

        std::cout << "Point1(" << point1.toString() << ")이 Box1에 포함: "
            << (box1.contains(point1) ? "예" : "아니오") << std::endl;
        std::cout << "Point2(" << point2.toString() << ")이 Box1에 포함: "
            << (box1.contains(point2) ? "예" : "아니오") << std::endl;
    }

    // OBB 충돌 테스트
    void testOBBCollision() {
        std::cout << "\n=== OBB 충돌 테스트 ===" << std::endl;

        // OBB 객체 생성
        Vector3 center1(0.0f, 0.0f, 0.0f);
        Vector3 extents1(1.0f, 1.0f, 1.0f);
        Matrix3x3 orientation1 = Matrix3x3::identity();
        OBB obb1(center1, extents1, orientation1);

        // 45도 회전된 OBB
        Vector3 center2(2.0f, 0.0f, 0.0f);
        Vector3 extents2(1.0f, 0.5f, 0.5f);
        Matrix3x3 orientation2 = Matrix3x3::rotationZ(M_PI / 4); // Z축 45도 회전
        OBB obb2(center2, extents2, orientation2);

        // 충돌하지 않는 OBB
        Vector3 center3(4.0f, 0.0f, 0.0f);
        Vector3 extents3(1.0f, 1.0f, 1.0f);
        Matrix3x3 orientation3 = Matrix3x3::identity();
        OBB obb3(center3, extents3, orientation3);

        std::cout << "OBB1: " << obb1.toString() << std::endl;
        std::cout << "OBB2: " << obb2.toString() << std::endl;
        std::cout << "OBB3: " << obb3.toString() << std::endl;

        // SAT 알고리즘을 사용한 충돌 감지
        bool collision12 = obb1.intersects(obb2);
        bool collision13 = obb1.intersects(obb3);
        bool collision23 = obb2.intersects(obb3);

        std::cout << "OBB1과 OBB2 충돌: " << (collision12 ? "예" : "아니오") << std::endl;
        std::cout << "OBB1과 OBB3 충돌: " << (collision13 ? "예" : "아니오") << std::endl;
        std::cout << "OBB2과 OBB3 충돌: " << (collision23 ? "예" : "아니오") << std::endl;

        // OBB에서 AABB로 변환
        AABB aabb1 = obb1.toAABB();
        std::cout << "OBB1을 포함하는 AABB: " << aabb1.toString() << std::endl;

        // OBB 회전 테스트
        Quaternion rotation = Quaternion::fromAxisAngle(Vector3(0.0f, 0.0f, 1.0f), M_PI / 2); // Z축 90도
        OBB rotatedObb = obb1;
        rotatedObb.rotate(rotation);
        std::cout << "Z축으로 90도 회전된 OBB1: " << rotatedObb.toString() << std::endl;
    }

    // 광역 단계(Broad Phase) 충돌 감지 성능 테스트
    void testBroadPhasePerformance() {
        std::cout << "\n=== 광역 단계 충돌 감지 성능 테스트 ===" << std::endl;

        // 테스트용 객체 생성
        const int numObjects = 1000;
        std::vector<Object3D> objects;
        objects.reserve(numObjects);

        // 랜덤 객체 생성
        srand(static_cast<unsigned int>(time(nullptr)));
        for (int i = 0; i < numObjects; i++) {
            Vector3 position(
                static_cast<float>(rand()) / RAND_MAX * 100.0f - 50.0f,
                static_cast<float>(rand()) / RAND_MAX * 100.0f - 50.0f,
                static_cast<float>(rand()) / RAND_MAX * 100.0f - 50.0f
            );

            Quaternion rotation = Quaternion::fromAxisAngle(
                Vector3::randomUnit(),
                static_cast<float>(rand()) / RAND_MAX * M_PI * 2.0f
            );

            Vector3 scale(
                0.5f + static_cast<float>(rand()) / RAND_MAX * 2.0f,
                0.5f + static_cast<float>(rand()) / RAND_MAX * 2.0f,
                0.5f + static_cast<float>(rand()) / RAND_MAX * 2.0f
            );

            Object3D obj;
            obj.setPosition(position);
            obj.setRotation(rotation);
            obj.setScale(scale);
            obj.updateBoundingVolumes();

            objects.push_back(obj);
        }

        // 일반 충돌 감지 (AABB)
        std::vector<std::pair<int, int>> collisions;

        auto startTime = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < numObjects; i++) {
            for (int j = i + 1; j < numObjects; j++) {
                if (objects[i].getAABB().intersects(objects[j].getAABB())) {
                    collisions.push_back(std::make_pair(i, j));
                }
            }
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

        std::cout << "일반 AABB 충돌 감지 시간: " << duration << "ms" << std::endl;
        std::cout << "감지된 충돌 쌍: " << collisions.size() << std::endl;

        // OpenMP 병렬화된 AABB 충돌 감지
        collisions.clear();
        std::vector<std::pair<int, int>> parallelCollisions;

        startTime = std::chrono::high_resolution_clock::now();

#pragma omp parallel
        {
            std::vector<std::pair<int, int>> localCollisions;

#pragma omp for
            for (int i = 0; i < numObjects; i++) {
                for (int j = i + 1; j < numObjects; j++) {
                    if (objects[i].getAABB().intersects(objects[j].getAABB())) {
                        localCollisions.push_back(std::make_pair(i, j));
                    }
                }
            }

#pragma omp critical
            {
                parallelCollisions.insert(parallelCollisions.end(), localCollisions.begin(), localCollisions.end());
            }
        }

        endTime = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

        std::cout << "OpenMP 병렬화 AABB 충돌 감지 시간: " << duration << "ms" << std::endl;
        std::cout << "감지된 충돌 쌍: " << parallelCollisions.size() << std::endl;

        // 스레드 수에 따른 성능 테스트
        std::cout << "\n=== 스레드 수에 따른 성능 테스트 ===" << std::endl;

        int maxThreads = omp_get_max_threads();
        for (int numThreads = 1; numThreads <= maxThreads; numThreads++) {
            collisions.clear();

            omp_set_num_threads(numThreads);
            startTime = std::chrono::high_resolution_clock::now();

#pragma omp parallel
            {
                std::vector<std::pair<int, int>> localCollisions;

#pragma omp for
                for (int i = 0; i < numObjects; i++) {
                    for (int j = i + 1; j < numObjects; j++) {
                        if (objects[i].getAABB().intersects(objects[j].getAABB())) {
                            localCollisions.push_back(std::make_pair(i, j));
                        }
                    }
                }

#pragma omp critical
                {
                    collisions.insert(collisions.end(), localCollisions.begin(), localCollisions.end());
                }
            }

            endTime = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

            std::cout << "스레드 " << numThreads << "개 사용 시간: " << duration << "ms" << std::endl;
        }

        // BVH 구축 및 충돌 감지
        std::cout << "\n=== BVH 기반 충돌 감지 테스트 ===" << std::endl;

        startTime = std::chrono::high_resolution_clock::now();

        BVH bvh;
        for (int i = 0; i < numObjects; i++) {
            bvh.insert(&objects[i]);
        }
        bvh.build();

        endTime = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

        std::cout << "BVH 구축 시간: " << duration << "ms" << std::endl;

        // BVH 기반 충돌 쿼리
        collisions.clear();

        startTime = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < numObjects; i++) {
            std::vector<Object3D*> potentialCollisions;
            bvh.query(&objects[i], potentialCollisions);

            for (Object3D* other : potentialCollisions) {
                int j = other - &objects[0];
                if (i < j) {  // 중복 검사 방지
                    collisions.push_back(std::make_pair(i, j));
                }
            }
        }

        endTime = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

        std::cout << "BVH 기반 충돌 쿼리 시간: " << duration << "ms" << std::endl;
        std::cout << "감지된 충돌 쌍: " << collisions.size() << std::endl;
    }

    // 광역-협역 파이프라인 테스트
    void testCollisionPipeline() {
        std::cout << "\n=== 충돌 감지 파이프라인 테스트 ===" << std::endl;

        // 테스트용 객체 생성
        const int numObjects = 100;
        std::vector<Object3D> objects;
        objects.reserve(numObjects);

        // 랜덤 객체 생성
        srand(static_cast<unsigned int>(time(nullptr)));
        for (int i = 0; i < numObjects; i++) {
            Vector3 position(
                static_cast<float>(rand()) / RAND_MAX * 50.0f - 25.0f,
                static_cast<float>(rand()) / RAND_MAX * 50.0f - 25.0f,
                static_cast<float>(rand()) / RAND_MAX * 50.0f - 25.0f
            );

            Quaternion rotation = Quaternion::fromAxisAngle(
                Vector3::randomUnit(),
                static_cast<float>(rand()) / RAND_MAX * M_PI * 2.0f
            );

            Vector3 scale(
                0.5f + static_cast<float>(rand()) / RAND_MAX * 2.0f,
                0.5f + static_cast<float>(rand()) / RAND_MAX * 2.0f,
                0.5f + static_cast<float>(rand()) / RAND_MAX * 2.0f
            );

            Object3D obj;
            obj.setPosition(position);
            obj.setRotation(rotation);
            obj.setScale(scale);
            obj.updateBoundingVolumes();

            objects.push_back(obj);
        }

        // 충돌 관리자 생성
        CollisionManager collisionManager;

        // 객체 등록
        for (auto& obj : objects) {
            collisionManager.addObject(&obj);
        }

        // 충돌 감지 실행
        auto startTime = std::chrono::high_resolution_clock::now();

        std::vector<std::pair<Object3D*, Object3D*>> collisions;
        collisionManager.detectCollisions(collisions);

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

        std::cout << "전체 충돌 파이프라인 실행 시간: " << duration << "ms" << std::endl;
        std::cout << "최종 감지된 충돌 쌍: " << collisions.size() << std::endl;

        // 각 단계별 시간 분석 (더미 데이터)
        std::cout << "\n=== 충돌 감지 단계별 시간 분석 ===" << std::endl;
        std::cout << "광역 단계(AABB): " << collisionManager.getBroadPhaseTime() << "ms" << std::endl;
        std::cout << "중간 단계(OBB): " << collisionManager.getMidPhaseTime() << "ms" << std::endl;
        std::cout << "협역 단계(GJK): " << collisionManager.getNarrowPhaseTime() << "ms" << std::endl;
    }
}

// 팀원 2 담당 부분 구현
namespace TeamMember2 {
    // 여기에 팀원 2 구현 코드 추가
}

int main() {
    std::cout << "===== 3D 객체 충돌 감지 프로젝트 테스트 =====\n" << std::endl;

    // 팀원 1 담당 부분 테스트
    TeamMember1::testVectorOperations();
    TeamMember1::testMatrixOperations();
    TeamMember1::testQuaternionOperations();
    TeamMember1::testAABBCollision();
    TeamMember1::testOBBCollision();
    TeamMember1::testBroadPhasePerformance();
    TeamMember1::testCollisionPipeline();

    // 팀원 2 담당 부분 테스트
    // TeamMember2::함수들 호출

    return 0;
}