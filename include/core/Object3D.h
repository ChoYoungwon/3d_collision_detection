#ifndef OBJECT3D_H
#define OBJECT3D_H

#include <vector>
#include <string>
#include <functional>
#include "Vector3.h"
#include "Matrix3x3.h"
#include "Quaternion.h"
#include "AABB.h"
#include "ConvexDecomposition.h"

class Object3D;

class CollisionInfo {
public:
    Object3D* otherObject;
    Vector3 contactPoint;
    Vector3 contactNormal;
    float penetrationDepth;

    CollisionInfo();
    CollisionInfo(Object3D* other, const Vector3& point, const Vector3& normal, float depth);
};

class Object3D {
public:
    // 변환 상태 (위치, 회전, 스케일)
    Vector3 position;
    Vector3 rotation; // 간단히 Euler 각도로 가정 (또는 필요에 따라 Quaternion 사용)
    Vector3 scale;
  
    Matrix3x3 transformMatrix;
    bool transformDirty;

    AABB localAABB;
    AABB worldAABB;
    bool aabbDirty;

    bool isInCollision;
    std::vector<CollisionInfo> collisions;

    // 메시 데이터
    std::vector<Vector3> vertices;
    std::vector<Vector3> normals;
    std::vector<int> indices;

    // 볼록 분해 결과
    std::vector<ConvexHull> convexHulls;
    bool isConvexDecomposed;

    // Callback function type for collision events
    using CollisionCallback = std::function<void(const CollisionInfo&)>;
    CollisionCallback onCollisionEnter;
    CollisionCallback onCollisionStay;
    CollisionCallback onCollisionExit;

public:
    Object3D(const std::string& _name = "Object");
    ~Object3D();

    // Position methods
    const Vector3& getPosition() const;
    void setPosition(const Vector3& pos);
    void translate(const Vector3& offset);

    // Rotation methods
    const Quaternion& getRotation() const;
    void setRotation(const Quaternion& rot);
    void rotate(const Quaternion& rot);
    void rotateAxis(const Vector3& axis, float angleRadians);

    // Scale methods
    const Vector3& getScale() const;
    void setScale(const Vector3& s);
    void setScale(float uniformScale);

    // Transform matrix operations
    const Matrix3x3& getTransformMatrix();
    void updateTransformMatrix();

    // AABB operations
    void setLocalAABB(const AABB& aabb);
    const AABB& getLocalAABB() const;
    const AABB& getAABB();
    void updateWorldAABB();

    // Transform operations
    Vector3 transformPoint(const Vector3& point);
    Vector3 transformDirection(const Vector3& dir);

    // Collision management
    bool isColliding() const;
    const std::vector<CollisionInfo>& getCollisions() const;
    void addCollision(const CollisionInfo& collision);
    void removeCollision(Object3D* other);
    void clearCollisions();

    // Collision event callbacks
    void setOnCollisionEnter(const CollisionCallback& callback);
    void setOnCollisionStay(const CollisionCallback& callback);
    void setOnCollisionExit(const CollisionCallback& callback);

    // Mesh data operations
    void setMeshData(const std::vector<Vector3>& verts, 
                    const std::vector<Vector3>& norms,
                    const std::vector<int>& inds);

    // File loading operations
    bool loadFromObjFile(const std::string& filepa th);

    // 볼록 분해 관련 메서드 (추가됨)
    bool computeConvexDecomposition(const VHACDParameters& params = VHACDParameters());
    bool loadConvexDecomposition(const std::string& filepath);
    void setConvexHulls(const std::vector<ConvexHull>& hulls);

    // GJK support function
    Vector3 getSupportPoint(const Vector3& direction) const;

    // Accessors
    bool isDecomposed() const;
    const std::vector<ConvexHull>& getConvexHulls() const;
    const std::vector<Vector3>& getVertices() const;
    const std::vector<Vector3>& getNormals() const;
    const std::vector<int>& getIndices() const;
    const std::string& getName() const;
    void setName(const std::string& _name);

    // Update method
    void update();
};

#endif
