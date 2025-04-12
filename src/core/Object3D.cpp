// src/core/Object3D.cpp
#include "core/Object3D.h"
#include "math/Matrix4x4.h" // 필요한 경우 이 헤더 파일도 구현해야 함

namespace Core {

    Object3D::Object3D()
        : m_position(0.0f, 0.0f, 0.0f),
        m_rotation(),
        m_scale(1.0f, 1.0f, 1.0f),
        m_boundingVolumeDirty(true) {
    }

    Object3D::Object3D(const Math::Vector3& position, const Math::Quaternion& rotation, const Math::Vector3& scale)
        : m_position(position),
        m_rotation(rotation),
        m_scale(scale),
        m_boundingVolumeDirty(true) {
    }

    const Math::Vector3& Object3D::getPosition() const {
        return m_position;
    }

    void Object3D::setPosition(const Math::Vector3& position) {
        m_position = position;
        m_boundingVolumeDirty = true;
    }

    const Math::Quaternion& Object3D::getRotation() const {
        return m_rotation;
    }

    void Object3D::setRotation(const Math::Quaternion& rotation) {
        m_rotation = rotation;
        m_boundingVolumeDirty = true;
    }

    const Math::Vector3& Object3D::getScale() const {
        return m_scale;
    }

    void Object3D::setScale(const Math::Vector3& scale) {
        m_scale = scale;
        m_boundingVolumeDirty = true;
    }

    void Object3D::translate(const Math::Vector3& translation) {
        m_position = m_position + translation;
        m_boundingVolumeDirty = true;
    }

    void Object3D::rotate(const Math::Quaternion& rotation) {
        m_rotation = rotation * m_rotation;
        m_boundingVolumeDirty = true;
    }

    void Object3D::scale(const Math::Vector3& scale) {
        m_scale.x *= scale.x;
        m_scale.y *= scale.y;
        m_scale.z *= scale.z;
        m_boundingVolumeDirty = true;
    }

    const Geometry::AABB& Object3D::getAABB() const {
        if (m_boundingVolumeDirty) {
            const_cast<Object3D*>(this)->updateBoundingVolumes();
        }
        return m_aabb;
    }

    const Geometry::OBB& Object3D::getOBB() const {
        if (m_boundingVolumeDirty) {
            const_cast<Object3D*>(this)->updateBoundingVolumes();
        }
        return m_obb;
    }

    void Object3D::updateBoundingVolumes() {
        // 이 메서드는 실제 메시 데이터를 기반으로 AABB와 OBB를 계산해야 함
        // 여기서는 간단한 구현만 제공

        // 예: 객체의 메시 정점을 기반으로 AABB 계산
        // ... (실제 메시 데이터 접근 및 처리 코드)

        // 예시: 간단한 AABB 생성 (실제로는 메시 데이터 필요)
        Math::Vector3 minPoint(-1.0f, -1.0f, -1.0f);
        Math::Vector3 maxPoint(1.0f, 1.0f, 1.0f);

        // 스케일 적용
        minPoint = minPoint * m_scale;
        maxPoint = maxPoint * m_scale;

        // AABB 생성
        m_aabb = Geometry::AABB(minPoint + m_position, maxPoint + m_position);

        // OBB 생성 (AABB를 기반으로 하되 회전 적용)
        m_obb = Geometry::OBB(m_position, (maxPoint - minPoint) * 0.5f, m_rotation);

        m_boundingVolumeDirty = false;
    }

    Math::Matrix4x4 Object3D::getWorldMatrix() const {
        // 이 메서드는 위치, 회전, 크기를 결합한 변환 행렬 반환
        // Matrix4x4 클래스가 구현되어 있어야 합니다

        // 예시 구현 (실제 Matrix4x4 구현에 따라 달라질 수 있음)
        return Math::Matrix4x4::createTransformation(m_position, m_rotation, m_scale);
    }

} // namespace Core