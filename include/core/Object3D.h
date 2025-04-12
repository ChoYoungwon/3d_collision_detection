// include/core/Object3D.h
#pragma once

#include "math/Vector3.h"
#include "math/Quaternion.h"
#include "geometry/AABB.h"
#include "geometry/OBB.h"

namespace Core {

    class Object3D {
    public:
        Object3D();
        Object3D(const Math::Vector3& position, const Math::Quaternion& rotation = Math::Quaternion(),
            const Math::Vector3& scale = Math::Vector3(1.0f, 1.0f, 1.0f));

        // 기본 getter/setter
        const Math::Vector3& getPosition() const;
        void setPosition(const Math::Vector3& position);

        const Math::Quaternion& getRotation() const;
        void setRotation(const Math::Quaternion& rotation);

        const Math::Vector3& getScale() const;
        void setScale(const Math::Vector3& scale);

        // 객체 변환 메서드
        void translate(const Math::Vector3& translation);
        void rotate(const Math::Quaternion& rotation);
        void scale(const Math::Vector3& scale);

        // 바운딩 볼륨 관련 메서드
        const Geometry::AABB& getAABB() const;
        const Geometry::OBB& getOBB() const;

        // 바운딩 볼륨 업데이트 (객체 변환 시 호출)
        void updateBoundingVolumes();

        // 월드 좌표계에서의 변환 행렬 계산
        Math::Matrix4x4 getWorldMatrix() const;

    private:
        Math::Vector3 m_position;        // 위치
        Math::Quaternion m_rotation;     // 회전
        Math::Vector3 m_scale;           // 크기

        Geometry::AABB m_aabb;           // 축 정렬 바운딩 박스
        Geometry::OBB m_obb;             // 방향성 바운딩 박스

        bool m_boundingVolumeDirty;      // 바운딩 볼륨 업데이트 필요 여부
    };

} // namespace Core