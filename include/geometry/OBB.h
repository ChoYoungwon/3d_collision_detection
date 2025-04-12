// include/geometry/OBB.h
#ifndef OBB_H
#define OBB_H

#include "../math/Vector3.h"
#include "../math/Matrix3x3.h"
#include "../math/Quaternion.h"
#include "AABB.h"
#include <vector>
#include <array>

class OBB {
public:
    Vector3 center;      // �߽���
    Vector3 halfExtents; // �� �� ���������� ���� ũ��
    Matrix3x3 orientation; // ���� (ȸ�� ���)

    // ������
    OBB() : center(0, 0, 0), halfExtents(1, 1, 1) {
        // �⺻ ������ �׵� ��� (�� ����)
        orientation = Matrix3x3();
    }

    OBB(const Vector3& center, const Vector3& halfExtents)
        : center(center), halfExtents(halfExtents) {
        // �⺻ ������ �׵� ��� (�� ����)
        orientation = Matrix3x3();
    }

    OBB(const Vector3& center, const Vector3& halfExtents, const Matrix3x3& orientation)
        : center(center), halfExtents(halfExtents), orientation(orientation) {
    }

    // AABB�κ��� OBB ����
    OBB(const AABB& aabb) {
        center = aabb.getCenter();
        halfExtents = aabb.getSize() * 0.5f;
        orientation = Matrix3x3(); // �׵� ��� (�� ����)
    }

    // ������ �������κ��� OBB ���� (�ּ��� �м� ���)
    // ������ ������ ���� �� ���� ��� ���
    OBB(const std::vector<Vector3>& points) {
        if (points.empty()) {
            center = Vector3(0, 0, 0);
            halfExtents = Vector3(0, 0, 0);
            orientation = Matrix3x3();
            return;
        }

        // �ּ��� �м��� �����ϹǷ� ���⼭�� ������ AABB�� ��ȯ
        AABB aabb(points);
        center = aabb.getCenter();
        halfExtents = aabb.getSize() * 0.5f;
        orientation = Matrix3x3(); // �׵� ��� (�� ����)
    }

    // OBB�� 8�� ������ ���ϱ�
    std::array<Vector3, 8> getCorners() const {
        std::array<Vector3, 8> corners;

        // ���� ������ ������ ���
        corners[0] = Vector3(-halfExtents.x, -halfExtents.y, -halfExtents.z);
        corners[1] = Vector3(halfExtents.x, -halfExtents.y, -halfExtents.z);
        corners[2] = Vector3(halfExtents.x, halfExtents.y, -halfExtents.z);
        corners[3] = Vector3(-halfExtents.x, halfExtents.y, -halfExtents.z);
        corners[4] = Vector3(-halfExtents.x, -halfExtents.y, halfExtents.z);
        corners[5] = Vector3(halfExtents.x, -halfExtents.y, halfExtents.z);
        corners[6] = Vector3(halfExtents.x, halfExtents.y, halfExtents.z);
        corners[7] = Vector3(-halfExtents.x, halfExtents.y, halfExtents.z);

        // ���� �������� ��ȯ
        for (auto& corner : corners) {
            corner = orientation * corner + center;
        }

        return corners;
    }

    // OBB�� �� ���ϱ�
    std::array<Vector3, 3> getAxes() const {
        std::array<Vector3, 3> axes;

        // �� ���� ���� ��Ÿ��
        axes[0] = Vector3(orientation(0, 0), orientation(0, 1), orientation(0, 2));
        axes[1] = Vector3(orientation(1, 0), orientation(1, 1), orientation(1, 2));
        axes[2] = Vector3(orientation(2, 0), orientation(2, 1), orientation(2, 2));

        return axes;
    }

    // OBB�� AABB�� ��ȯ (��踦 �����ϴ� ���� ���� AABB)
    AABB toAABB() const {
        std::array<Vector3, 8> corners = getCorners();
        return AABB(std::vector<Vector3>(corners.begin(), corners.end()));
    }

    // ���� OBB ���ο� �ִ��� Ȯ��
    bool contains(const Vector3& point) const {
        // ���� OBB�� ���� �������� ��ȯ
        Vector3 localPoint = orientation.transpose() * (point - center);

        // ���� �������� ���� Ȯ��
        return std::abs(localPoint.x) <= halfExtents.x &&
            std::abs(localPoint.y) <= halfExtents.y &&
            std::abs(localPoint.z) <= halfExtents.z;
    }

    // OBB �浹 �˻� (�и��� ���� ���)
    bool intersects(const OBB& other) const {
        // 15���� �и��� �׽�Ʈ �ʿ�
        // - �� OBB�� �� 3�� �� (�� 6��)
        // - �� OBB ���� ���� (�� 9��)

        // OBB A�� ��
        std::array<Vector3, 3> axesA = getAxes();
        // OBB B�� ��
        std::array<Vector3, 3> axesB = other.getAxes();

        // OBB�� �߽� �� ����
        Vector3 t = other.center - center;

        // �� OBB�� ������ �׽�Ʈ
        for (int i = 0; i < 3; i++) {
            if (!overlapOnAxis(other, axesA[i], t)) {
                return false;
            }
        }

        for (int i = 0; i < 3; i++) {
            if (!overlapOnAxis(other, axesB[i], t)) {
                return false;
            }
        }

        // �� OBB�� �� �� �������� �׽�Ʈ
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Vector3 axis = Vector3::cross(axesA[i], axesB[j]);
                // ������ 0�̸� ���� �����ϹǷ� ����
                if (axis.lengthSquared() < 1e-6f) {
                    continue;
                }
                axis.normalize();

                if (!overlapOnAxis(other, axis, t)) {
                    return false;
                }
            }
        }

        // ��� �࿡�� ��ħ�� ������ �浹
        return true;
    }

private:
    // Ư�� �࿡ ���� OBB ���� ��ħ �˻�
    bool overlapOnAxis(const OBB& other, const Vector3& axis, const Vector3& t) const {
        // �� OBB�� �࿡ ���� ���������� ���
        float ra = projectExtent(axis);
        float rb = other.projectExtent(axis);

        // �߽� �� �Ÿ��� ���� ���
        float t_projection = std::abs(Vector3::dot(t, axis));

        // �࿡ ���� ��ħ �˻�
        return t_projection <= (ra + rb);
    }

    // OBB�� ���������� ���
    float projectExtent(const Vector3& axis) const {
        // �� ���� �࿡ ���� ���� �ջ�
        std::array<Vector3, 3> obb_axes = getAxes();
        return std::abs(Vector3::dot(halfExtents.x * obb_axes[0], axis)) +
            std::abs(Vector3::dot(halfExtents.y * obb_axes[1], axis)) +
            std::abs(Vector3::dot(halfExtents.z * obb_axes[2], axis));
    }
};

#endif // OBB_H