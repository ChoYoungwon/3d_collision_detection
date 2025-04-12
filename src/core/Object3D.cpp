// src/core/Object3D.cpp
#include "core/Object3D.h"
#include "math/Matrix4x4.h" // �ʿ��� ��� �� ��� ���ϵ� �����ؾ� ��

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
        // �� �޼���� ���� �޽� �����͸� ������� AABB�� OBB�� ����ؾ� ��
        // ���⼭�� ������ ������ ����

        // ��: ��ü�� �޽� ������ ������� AABB ���
        // ... (���� �޽� ������ ���� �� ó�� �ڵ�)

        // ����: ������ AABB ���� (�����δ� �޽� ������ �ʿ�)
        Math::Vector3 minPoint(-1.0f, -1.0f, -1.0f);
        Math::Vector3 maxPoint(1.0f, 1.0f, 1.0f);

        // ������ ����
        minPoint = minPoint * m_scale;
        maxPoint = maxPoint * m_scale;

        // AABB ����
        m_aabb = Geometry::AABB(minPoint + m_position, maxPoint + m_position);

        // OBB ���� (AABB�� ������� �ϵ� ȸ�� ����)
        m_obb = Geometry::OBB(m_position, (maxPoint - minPoint) * 0.5f, m_rotation);

        m_boundingVolumeDirty = false;
    }

    Math::Matrix4x4 Object3D::getWorldMatrix() const {
        // �� �޼���� ��ġ, ȸ��, ũ�⸦ ������ ��ȯ ��� ��ȯ
        // Matrix4x4 Ŭ������ �����Ǿ� �־�� �մϴ�

        // ���� ���� (���� Matrix4x4 ������ ���� �޶��� �� ����)
        return Math::Matrix4x4::createTransformation(m_position, m_rotation, m_scale);
    }

} // namespace Core