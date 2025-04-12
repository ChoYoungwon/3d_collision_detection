#pragma once

#include <vector>
#include <string>
#include <functional>
#include "Vector3.h"
#include "Matrix3x3.h"
#include "Quaternion.h"
#include "AABB.h"

class Object3D;

class CollisionInfo {
public:
    Object3D* otherObject;
    Vector3 contactPoint;
    Vector3 contactNormal;
    float penetrationDepth;

    CollisionInfo() : otherObject(nullptr), penetrationDepth(0.0f) {}

    CollisionInfo(Object3D* other, const Vector3& point, const Vector3& normal, float depth)
        : otherObject(other), contactPoint(point), contactNormal(normal), penetrationDepth(depth) {}
};

class Object3D {
private:
    std::string name;
    Vector3 position;
    Quaternion rotation;
    Vector3 scale;

    Matrix3x3 transformMatrix;
    bool transformDirty;

    AABB localAABB;
    AABB worldAABB;
    bool aabbDirty;

    bool isInCollision;
    std::vector<CollisionInfo> collisions;

    // Callback function type for collision events
    using CollisionCallback = std::function<void(const CollisionInfo&)>;
    CollisionCallback onCollisionEnter;
    CollisionCallback onCollisionStay;
    CollisionCallback onCollisionExit;

public:
    Object3D(const std::string& _name = "Object")
        : name(_name),
        position(Vector3(0, 0, 0)),
        rotation(Quaternion::identity()),
        scale(Vector3(1, 1, 1)),
        transformDirty(true),
        aabbDirty(true),
        isInCollision(false) {

        // Default local AABB (unit cube centered at origin)
        localAABB.min = Vector3(-0.5f, -0.5f, -0.5f);
        localAABB.max = Vector3(0.5f, 0.5f, 0.5f);

        updateTransformMatrix();
        updateWorldAABB();
    }

    // Position methods
    const Vector3& getPosition() const {
        return position;
    }

    void setPosition(const Vector3& pos) {
        position = pos;
        transformDirty = true;
        aabbDirty = true;
    }

    void translate(const Vector3& offset) {
        position += offset;
        transformDirty = true;
        aabbDirty = true;
    }

    // Rotation methods
    const Quaternion& getRotation() const {
        return rotation;
    }

    void setRotation(const Quaternion& rot) {
        rotation = rot;
        transformDirty = true;
        aabbDirty = true;
    }

    void rotate(const Quaternion& rot) {
        rotation = rotation * rot;
        transformDirty = true;
        aabbDirty = true;
    }

    void rotateAxis(const Vector3& axis, float angleRadians) {
        Quaternion q = Quaternion::fromAxisAngle(axis, angleRadians);
        rotate(q);
    }

    // Scale methods
    const Vector3& getScale() const {
        return scale;
    }

    void setScale(const Vector3& s) {
        scale = s;
        transformDirty = true;
        aabbDirty = true;
    }

    void setScale(float uniformScale) {
        scale = Vector3(uniformScale, uniformScale, uniformScale);
        transformDirty = true;
        aabbDirty = true;
    }

    // Transform matrix operations
    const Matrix3x3& getTransformMatrix() {
        if (transformDirty) {
            updateTransformMatrix();
        }
        return transformMatrix;
    }

    void updateTransformMatrix() {
        // Create rotation matrix from quaternion
        Matrix3x3 rotMat = rotation.toMatrix3x3();

        // Apply scale
        rotMat[0][0] *= scale.x;
        rotMat[1][0] *= scale.x;
        rotMat[2][0] *= scale.x;

        rotMat[0][1] *= scale.y;
        rotMat[1][1] *= scale.y;
        rotMat[2][1] *= scale.y;

        rotMat[0][2] *= scale.z;
        rotMat[1][2] *= scale.z;
        rotMat[2][2] *= scale.z;

        // Set translation components
        rotMat[0][3] = position.x;
        rotMat[1][3] = position.y;
        rotMat[2][3] = position.z;

        transformMatrix = rotMat;
        transformDirty = false;
    }

    // AABB operations
    void setLocalAABB(const AABB& aabb) {
        localAABB = aabb;
        aabbDirty = true;
    }

    const AABB& getLocalAABB() const {
        return localAABB;
    }

    const AABB& getAABB() {
        if (aabbDirty) {
            updateWorldAABB();
        }
        return worldAABB;
    }

    void updateWorldAABB() {
        if (transformDirty) {
            updateTransformMatrix();
        }

        // Initialize with transformed corners
        Vector3 corners[8];
        corners[0] = transformPoint(Vector3(localAABB.min.x, localAABB.min.y, localAABB.min.z));
        corners[1] = transformPoint(Vector3(localAABB.max.x, localAABB.min.y, localAABB.min.z));
        corners[2] = transformPoint(Vector3(localAABB.min.x, localAABB.max.y, localAABB.min.z));
        corners[3] = transformPoint(Vector3(localAABB.max.x, localAABB.max.y, localAABB.min.z));
        corners[4] = transformPoint(Vector3(localAABB.min.x, localAABB.min.y, localAABB.max.z));
        corners[5] = transformPoint(Vector3(localAABB.max.x, localAABB.min.y, localAABB.max.z));
        corners[6] = transformPoint(Vector3(localAABB.min.x, localAABB.max.y, localAABB.max.z));
        corners[7] = transformPoint(Vector3(localAABB.max.x, localAABB.max.y, localAABB.max.z));

        // Find min and max points
        worldAABB.min = corners[0];
        worldAABB.max = corners[0];

        for (int i = 1; i < 8; ++i) {
            worldAABB.min.x = std::min(worldAABB.min.x, corners[i].x);
            worldAABB.min.y = std::min(worldAABB.min.y, corners[i].y);
            worldAABB.min.z = std::min(worldAABB.min.z, corners[i].z);

            worldAABB.max.x = std::max(worldAABB.max.x, corners[i].x);
            worldAABB.max.y = std::max(worldAABB.max.y, corners[i].y);
            worldAABB.max.z = std::max(worldAABB.max.z, corners[i].z);
        }

        aabbDirty = false;
    }

    Vector3 transformPoint(const Vector3& point) {
        if (transformDirty) {
            updateTransformMatrix();
        }

        Vector3 result;
        result.x = transformMatrix[0][0] * point.x + transformMatrix[0][1] * point.y +
            transformMatrix[0][2] * point.z + transformMatrix[0][3];
        result.y = transformMatrix[1][0] * point.x + transformMatrix[1][1] * point.y +
            transformMatrix[1][2] * point.z + transformMatrix[1][3];
        result.z = transformMatrix[2][0] * point.x + transformMatrix[2][1] * point.y +
            transformMatrix[2][2] * point.z + transformMatrix[2][3];
        return result;
    }

    Vector3 transformDirection(const Vector3& dir) {
        if (transformDirty) {
            updateTransformMatrix();
        }

        Vector3 result;
        result.x = transformMatrix[0][0] * dir.x + transformMatrix[0][1] * dir.y + transformMatrix[0][2] * dir.z;
        result.y = transformMatrix[1][0] * dir.x + transformMatrix[1][1] * dir.y + transformMatrix[1][2] * dir.z;
        result.z = transformMatrix[2][0] * dir.x + transformMatrix[2][1] * dir.y + transformMatrix[2][2] * dir.z;
        return result;
    }

    // Collision management
    bool isColliding() const {
        return isInCollision;
    }

    const std::vector<CollisionInfo>& getCollisions() const {
        return collisions;
    }

    void addCollision(const CollisionInfo& collision) {
        // Check if already colliding with this object
        for (const auto& existing : collisions) {
            if (existing.otherObject == collision.otherObject) {
                return; // Already colliding with this object
            }
        }

        // Add new collision
        collisions.push_back(collision);

        // If this is the first collision, trigger onCollisionEnter
        if (!isInCollision) {
            isInCollision = true;
            if (onCollisionEnter) {
                onCollisionEnter(collision);
            }
        }
        else {
            // Otherwise trigger onCollisionStay
            if (onCollisionStay) {
                onCollisionStay(collision);
            }
        }
    }

    void removeCollision(Object3D* other) {
        auto it = std::find_if(collisions.begin(), collisions.end(),
            [other](const CollisionInfo& info) { return info.otherObject == other; });

        if (it != collisions.end()) {
            // Trigger onCollisionExit callback
            if (onCollisionExit) {
                onCollisionExit(*it);
            }

            // Remove the collision
            collisions.erase(it);

            // Update collision state
            isInCollision = !collisions.empty();
        }
    }

    void clearCollisions() {
        // Trigger onCollisionExit for all active collisions
        if (onCollisionExit) {
            for (const auto& collision : collisions) {
                onCollisionExit(collision);
            }
        }

        collisions.clear();
        isInCollision = false;
    }

    // Collision event callbacks
    void setOnCollisionEnter(const CollisionCallback& callback) {
        onCollisionEnter = callback;
    }

    void setOnCollisionStay(const CollisionCallback& callback) {
        onCollisionStay = callback;
    }

    void setOnCollisionExit(const CollisionCallback& callback) {
        onCollisionExit = callback;
    }

    // Basic accessors
    const std::string& getName() const {
        return name;
    }

    void setName(const std::string& _name) {
        name = _name;
    }

    void update() {
        if (transformDirty) {
            updateTransformMatrix();
        }

        if (aabbDirty) {
            updateWorldAABB();
        }
    }
};