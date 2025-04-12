// include/core/Object3D.h
#ifndef OBJECT3D_H
#define OBJECT3D_H

#include "../math/Vector3.h"
#include "../math/Matrix3x3.h"
#include "../math/Quaternion.h"
#include "../geometry/AABB.h"
#include "../geometry/OBB.h"
#include <string>
#include <vector>
#include <memory>

class Object3D {
public:
    std::string name;           // 객체 이름
    Vector3 position;           // 위치
    Quaternion rotation;        // 회전
    Vector3 scale;              // 스케일

    std::vector<Vector3> vertices;  // 정점 데이터
    std::vector<int> indices;       // 인덱스 데이터 (삼각형)

    AABB aabb;                  // 축 정렬 경계 상자
    OBB obb;                    // 방향성 경계 상자

    bool isDirty;               // 변환이 변경되었는지 여부

    // 생성자
    Object3D() :
        position(0, 0, 0),
        rotation(),  // 기본 항등 회전
        scale(1, 1, 1),
        isDirty(true) {}

    Object3D(const std::string& name) :
        name(name),
        position(0, 0, 0),
        rotation(),  // 기본 항등 회전
        scale(1, 1, 1),
        isDirty(true) {}

    // 변환 메서드
    void setPosition(const Vector3& pos) {
        position = pos;
        isDirty = true;
    }

    void setRotation(const Quaternion& rot) {
        rotation = rot;
        isDirty = true;
    }

    void setScale(const Vector3& s) {
        scale = s;
        isDirty = true;
    }

    void translate(const Vector3& offset) {
        position += offset;
        isDirty = true;
    }

    void rotate(const Quaternion& q) {
        rotation = q * rotation;
        isDirty = true;
    }

    void rotateX(float angleRadians) {
        Quaternion q = Quaternion::fromAxisAngle(Vector3(1, 0, 0), angleRadians);
        rotate(q);
    }

    void rotateY(float angleRadians) {
        Quaternion q = Quaternion::fromAxisAngle(Vector3(0, 1, 0), angleRadians);
        rotate(q);
    }

    void rotateZ(float angleRadians) {
        Quaternion q = Quaternion::fromAxisAngle(Vector3(0, 0, 1), angleRadians);
        rotate(q);
    }

    // 정점 데이터 설정
    void setVertices(const std::vector<Vector3>& verts) {
        vertices = verts;
        updateBoundingVolumes();
    }

    void setIndices(const std::vector<int>& inds) {
        indices = inds;
    }

    // 월드 변환 행렬 계산
    Matrix3x3 getWorldMatrix() const {
        Matrix3x3 R = rotation.toMatrix3x3();
        Matrix3x3 S = Matrix3x3::createScale(scale);
        return R * S;
    }

    // 로컬에서 월드 좌표계로 변환
    Vector3 localToWorld(const Vector3& localPoint) const {
        Matrix3x3 worldMatrix = getWorldMatrix();
        return worldMatrix * localPoint + position;
    }

    // 월드에서 로컬 좌표계로 변환
    Vector3 worldToLocal(const Vector3& worldPoint) const {
        Matrix3x3 worldMatrix = getWorldMatrix();
        Matrix3x3 invWorldMatrix = worldMatrix.inverse();
        return invWorldMatrix * (worldPoint - position);
    }

    // 경계 볼륨 업데이트
    void updateBoundingVolumes() {
        if (vertices.empty()) {
            aabb = AABB(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
            obb = OBB(Vector3(0, 0, 0), Vector3(0.5f, 0.5f, 0.5f));
            return;
        }

        // 로컬 공간의 AABB 계산
        aabb = AABB(vertices);

        // OBB는 AABB를 기반으로 생성 (단순화)
        obb = OBB(aabb.getCenter(), aabb.getSize() * 0.5f);

        // 변환 정보 반영
        updateWorldBoundingVolumes();

        isDirty = false;
    }

    // 월드 공간의 경계 볼륨 업데이트
    void updateWorldBoundingVolumes() {
        // 월드 공간의 정점 계산
        std::vector<Vector3> worldVertices;
        worldVertices.reserve(vertices.size());

        for (const auto& vertex : vertices) {
            worldVertices.push_back(localToWorld(vertex));
        }

        // 월드 공간의 AABB 계산
        aabb = AABB(worldVertices);

        // 월드 공간의 OBB 생성
        Matrix3x3 worldMatrix = rotation.toMatrix3x3();
        obb = OBB(position, aabb.getSize() * 0.5f * scale, worldMatrix);
    }

    // 충돌 감지 (AABB)
    bool intersectsAABB(const Object3D& other) const {
        // AABB가 변경되었다면 업데이트
        if (isDirty || other.isDirty) {
            const_cast<Object3D*>(this)->updateWorldBoundingVolumes();
            const_cast<Object3D*>(&other)->updateWorldBoundingVolumes();
        }

        return aabb.intersects(other.aabb);
    }

    // 충돌 감지 (OBB)
    bool intersectsOBB(const Object3D& other) const {
        // OBB가 변경되었다면 업데이트
        if (isDirty || other.isDirty) {
            const_cast<Object3D*>(this)->updateWorldBoundingVolumes();
            const_cast<Object3D*>(&other)->updateWorldBoundingVolumes();
        }

        return obb.intersects(other.obb);
    }
};

#endif // OBJECT3D_H