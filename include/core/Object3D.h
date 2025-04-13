#ifndef OBJECT3D_H
#define OBJECT3D_H

#include "../math/Vector3.h"
#include "../geometry/AABB.h"  // AABB가 Geometry 네임스페이스에 포함되어 있다고 가정

namespace Core {

class Object3D {
public:
    // 변환 상태 (위치, 회전, 스케일)
    Vector3 position;
    Vector3 rotation; // 간단히 Euler 각도로 가정 (또는 필요에 따라 Quaternion 사용)
    Vector3 scale;

    // 충돌 판정을 위한 경계 부피 (AABB)
    AABB aabb;

    // 상태 플래그 : true면 변환 혹은 속성이 변경되어 현재 경계 부피가 오래되었음을 의미
    bool isDirty;

    Object3D()
        : position(0, 0, 0),
          rotation(0, 0, 0),
          scale(1, 1, 1),
          isDirty(true),
          aabb() // AABB의 기본 생성자 호출 (최대/최소값 초기화)
    {
    }

    virtual ~Object3D() {}

    // 현재 객체의 변환 상태를 반영하여 경계 부피(AABB)를 업데이트합니다.
    // 실제 구현에서는 로컬 기하학 데이터를 변환행렬에 따라 변환한 후 AABB를 재계산해야 합니다.
    virtual void updateBoundingVolumes() {
        // 예시: 간단히 aabb를 재설정하고, isDirty를 false로 변경합니다.
        aabb = AABB();
        // 실제 코드에서는 Object3D가 보유한 로컬 정점들에 대해 변환을 적용한 후 AABB를 계산해야 합니다.
        isDirty = false;
    }

    // 경계 부피 AABB를 반환합니다.
    virtual const AABB& getAABB() const {
        return aabb;
    }
};

} // namespace Core

#endif // OBJECT3D_H
