#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include "Object3D.h"
#include "SweepAndPrune.h"
#include "UniformGrid.h"
#include "Octree.h"
#include "BVH.h"

enum class BroadPhaseType {
    SWEEP_AND_PRUNE,
    UNIFORM_GRID,
    OCTREE,
    BVH
};

class CollisionManager {
private:
    // Collision detection algorithms
    std::unique_ptr<SweepAndPrune> sap;
    std::unique_ptr<UniformGrid> grid;
    std::unique_ptr<Octree> octree;
    std::unique_ptr<BVH> bvh;

    BroadPhaseType currentBroadPhase;
    std::vector<Object3D*> objects;

    // Collision state tracking
    std::unordered_map<Object3D*, std::vector<Object3D*>> currentCollisions;

    // Performance metrics
    double lastBroadPhaseTime;
    double lastNarrowPhaseTime;
    int potentialCollisionCount;
    int actualCollisionCount;

public:
    CollisionManager(BroadPhaseType broadPhaseType = BroadPhaseType::SWEEP_AND_PRUNE) {
        // Initialize broadphase algorithms with default settings
        sap = std::make_unique<SweepAndPrune>();

        // Default grid size
        Vector3 worldMin(-100, -100, -100);
        Vector3 worldMax(100, 100, 100);
        float cellSize = 10.0f;
        grid = std::make_unique<UniformGrid>(cellSize, worldMin, worldMax);

        // Default octree settings
        AABB worldBounds(worldMin, worldMax);
        octree = std::make_unique<Octree>(worldBounds);

        // BVH will be built when objects are added
        bvh = std::make_unique<BVH>();

        currentBroadPhase = broadPhaseType;

        // Initialize performance metrics
        lastBroadPhaseTime = 0.0;
        lastNarrowPhaseTime = 0.0;
        potentialCollisionCount = 0;
        actualCollisionCount = 0;
    }

    void setBroadPhaseType(BroadPhaseType type) {
        currentBroadPhase = type;

        // If changing to BVH, rebuild it
        if (type == BroadPhaseType::BVH) {
            bvh->build(objects);
        }
    }

    void addObject(Object3D* object) {
        objects.push_back(object);

        // Add to all broadphase algorithms
        sap->addObject(object);
        grid->addObject(object);
        octree->addObject(object);

        // Rebuild BVH when objects change
        if (currentBroadPhase == BroadPhaseType::BVH) {
            bvh->build(objects);
        }
    }

    void removeObject(Object3D* object) {
        // Remove from object list
        auto it = std::remove(objects.begin(), objects.end(), object);
        if (it != objects.end()) {
            objects.erase(it, objects.end());

            // Remove from all broadphase algorithms
            sap->removeObject(object);
            grid->removeObject(object);
            octree->removeObject(object);

            // Clear collision state for this object
            currentCollisions.erase(object);

            // Remove this object from all other objects' collision lists
            for (auto& entry : currentCollisions) {
                auto& collisionList = entry.second;
                auto colIt = std::remove(collisionList.begin(), collisionList.end(), object);
                if (colIt != collisionList.end()) {
                    collisionList.erase(colIt, collisionList.end());
                }

                // Also notify the object about this collision removal
                entry.first->removeCollision(object);
            }

            // Rebuild BVH when objects change
            if (currentBroadPhase == BroadPhaseType::BVH) {
                bvh->build(objects);
            }
        }
    }

    void updateObject(Object3D* object) {
        // Update object in all broadphase algorithms
        sap->updateObject(object);
        grid->updateObject(object);
        octree->updateObject(object);

        // For BVH, we need to rebuild or update the whole structure
        if (currentBroadPhase == BroadPhaseType::BVH) {
            bvh->update();
        }
    }

    void updateAllObjects() {
        for (Object3D* obj : objects) {
            obj->update();
            updateObject(obj);
        }
    }

    void detectCollisions() {
        // Clear collision counts for metrics
        potentialCollisionCount = 0;
        actualCollisionCount = 0;

        // Broad phase - find potential collisions
        std::vector<std::pair<Object3D*, Object3D*>> potentialCollisions;

        auto startBroadPhase = std::chrono::high_resolution_clock::now();

        switch (currentBroadPhase) {
        case BroadPhaseType::SWEEP_AND_PRUNE:
            sap->update();
            potentialCollisions = sap->getPotentialCollisions();
            break;

        case BroadPhaseType::UNIFORM_GRID:
            grid->findPotentialCollisions();
            potentialCollisions = grid->getPotentialCollisions();
            break;

        case BroadPhaseType::OCTREE:
            octree->findPotentialCollisions();
            potentialCollisions = octree->getPotentialCollisions();
            break;

        case BroadPhaseType::BVH:
            bvh->findPotentialCollisions();
            potentialCollisions = bvh->getPotentialCollisions();
            break;
        }

        auto endBroadPhase = std::chrono::high_resolution_clock::now();
        lastBroadPhaseTime = std::chrono::duration<double, std::milli>(endBroadPhase - startBroadPhase).count();

        potentialCollisionCount = potentialCollisions.size();

        // Store current collision state to detect exits
        std::unordered_map<Object3D*, std::vector<Object3D*>> newCollisions;

        // Narrow phase - Check actual collisions
        auto startNarrowPhase = std::chrono::high_resolution_clock::now();

        for (const auto& pair : potentialCollisions) {
            Object3D* objA = pair.first;
            Object3D* objB = pair.second;

            // Perform detailed collision test (placeholder for actual collision detection)
            CollisionInfo collisionInfo;
            bool isColliding = checkDetailedCollision(objA, objB, collisionInfo);

            if (isColliding) {
                // Record collision for both objects
                newCollisions[objA].push_back(objB);
                newCollisions[objB].push_back(objA);

                // Update collision info for both objects
                objA->addCollision(collisionInfo);

                // Create reversed collision info for objB
                CollisionInfo reversedInfo(objA, collisionInfo.contactPoint,
                    collisionInfo.contactNormal * -1.0f,
                    collisionInfo.penetrationDepth);
                objB->addCollision(reversedInfo);

                actualCollisionCount++;
            }
        }

        auto endNarrowPhase = std::chrono::high_resolution_clock::now();
        lastNarrowPhaseTime = std::chrono::duration<double, std::milli>(endNarrowPhase - startNarrowPhase).count();

        // Check for collision exits
        for (Object3D* obj : objects) {
            if (currentCollisions.find(obj) != currentCollisions.end()) {
                for (Object3D* other : currentCollisions[obj]) {
                    // If obj no longer collides with other, trigger exit
                    auto& newObjCollisions = newCollisions[obj];
                    if (std::find(newObjCollisions.begin(), newObjCollisions.end(), other) == newObjCollisions.end()) {
                        obj->removeCollision(other);
                    }
                }
            }
        }

        // Update current collision state
        currentCollisions = std::move(newCollisions);
    }

    // Performance metrics accessors
    double getBroadPhaseTime() const { return lastBroadPhaseTime; }
    double getNarrowPhaseTime() const { return lastNarrowPhaseTime; }
    double getTotalTime() const { return lastBroadPhaseTime + lastNarrowPhaseTime; }
    int getPotentialCollisionCount() const { return potentialCollisionCount; }
    int getActualCollisionCount() const { return actualCollisionCount; }

private:
    // Detailed collision check - placeholder for actual implementation
    bool checkDetailedCollision(Object3D* objA, Object3D* objB, CollisionInfo& outInfo) {
        // This is where you would call into the narrowphase collision system
        // For now we'll just use a simplified AABB test as a placeholder

        const AABB& aabbA = objA->getAABB();
        const AABB& aabbB = objB->getAABB();

        // Check if AABBs overlap
        if (aabbA.min.x <= aabbB.max.x && aabbA.max.x >= aabbB.min.x &&
            aabbA.min.y <= aabbB.max.y && aabbA.max.y >= aabbB.min.y &&
            aabbA.min.z <= aabbB.max.z && aabbA.max.z >= aabbB.min.z) {

            // Calculate penetration depth in each axis
            float xPenetration = std::min(aabbA.max.x - aabbB.min.x, aabbB.max.x - aabbA.min.x);
            float yPenetration = std::min(aabbA.max.y - aabbB.min.y, aabbB.max.y - aabbA.min.y);
            float zPenetration = std::min(aabbA.max.z - aabbB.min.z, aabbB.max.z - aabbA.min.z);

            // Find axis of minimum penetration
            Vector3 normal;
            float minPenetration;

            if (xPenetration <= yPenetration && xPenetration <= zPenetration) {
                minPenetration = xPenetration;
                normal = (aabbA.min.x + aabbA.max.x > aabbB.min.x + aabbB.max.x) ?
                    Vector3(1, 0, 0) : Vector3(-1, 0, 0);
            }
            else if (yPenetration <= zPenetration) {
                minPenetration = yPenetration;
                normal = (aabbA.min.y + aabbA.max.y > aabbB.min.y + aabbB.max.y) ?
                    Vector3(0, 1, 0) : Vector3(0, -1, 0);
            }
            else {
                minPenetration = zPenetration;
                normal = (aabbA.min.z + aabbA.max.z > aabbB.min.z + aabbB.max.z) ?
                    Vector3(0, 0, 1) : Vector3(0, 0, -1);
            }

            // Calculate approximate contact point
            Vector3 centerA = (aabbA.min + aabbA.max) * 0.5f;
            Vector3 centerB = (aabbB.min + aabbB.max) * 0.5f;
            Vector3 contactPoint = (centerA + centerB) * 0.5f;

            // Fill collision info
            outInfo.otherObject = objB;
            outInfo.contactPoint = contactPoint;
            outInfo.contactNormal = normal;
            outInfo.penetrationDepth = minPenetration;

            return true;
        }

        return false;
    }
};