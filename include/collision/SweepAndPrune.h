#pragma once

#include <vector>
#include <algorithm>
#include "Object3D.h"
#include "AABB.h"

class SweepAndPrune {
private:
    struct EndPoint {
        float value;
        Object3D* object;
        bool isMin;  // true for min endpoint, false for max endpoint

        EndPoint(float v, Object3D* obj, bool min) : value(v), object(obj), isMin(min) {}
    };

    std::vector<EndPoint> xEndpoints;
    std::vector<std::pair<Object3D*, Object3D*>> potentialCollisions;
    std::vector<Object3D*> activeObjects;

public:
    SweepAndPrune() {}

    void addObject(Object3D* object) {
        const AABB& aabb = object->getAABB();
        xEndpoints.emplace_back(aabb.min.x, object, true);
        xEndpoints.emplace_back(aabb.max.x, object, false);
        sortEndpoints();
    }

    void removeObject(Object3D* object) {
        auto it = std::remove_if(xEndpoints.begin(), xEndpoints.end(),
            [object](const EndPoint& ep) { return ep.object == object; });
        xEndpoints.erase(it, xEndpoints.end());
    }

    void updateObject(Object3D* object) {
        removeObject(object);
        addObject(object);
    }

    void sortEndpoints() {
        std::sort(xEndpoints.begin(), xEndpoints.end(),
            [](const EndPoint& a, const EndPoint& b) {
                if (a.value != b.value) return a.value < b.value;
                // If values are equal, min endpoints come before max endpoints
                return a.isMin && !b.isMin;
            });
    }

    void findPotentialCollisions() {
        potentialCollisions.clear();
        activeObjects.clear();

        for (const auto& endpoint : xEndpoints) {
            Object3D* currentObject = endpoint.object;

            if (endpoint.isMin) {
                // When we encounter a min endpoint, check against all active objects
                for (Object3D* obj : activeObjects) {
                    // Check if AABBs overlap in all three axes
                    if (checkAABBOverlap(currentObject->getAABB(), obj->getAABB())) {
                        potentialCollisions.emplace_back(currentObject, obj);
                    }
                }
                // Add the current object to active list
                activeObjects.push_back(currentObject);
            }
            else {
                // When we encounter a max endpoint, remove from active list
                auto it = std::find(activeObjects.begin(), activeObjects.end(), currentObject);
                if (it != activeObjects.end()) {
                    activeObjects.erase(it);
                }
            }
        }
    }

    bool checkAABBOverlap(const AABB& a, const AABB& b) {
        // Check overlap in all three axes (already confirmed for x-axis in SAP algorithm)
        return (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
            (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }

    const std::vector<std::pair<Object3D*, Object3D*>>& getPotentialCollisions() const {
        return potentialCollisions;
    }

    void update() {
        sortEndpoints();
        findPotentialCollisions();
    }
};