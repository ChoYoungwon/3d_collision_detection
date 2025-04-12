#pragma once

#include <vector>
#include <array>
#include <memory>
#include <unordered_set>
#include "Object3D.h"
#include "Vector3.h"
#include "AABB.h"

class Octree {
private:
    struct OctreeNode {
        AABB bounds;
        std::vector<Object3D*> objects;
        std::array<std::unique_ptr<OctreeNode>, 8> children;
        bool isLeaf;
        int depth;

        OctreeNode(const AABB& _bounds, int _depth)
            : bounds(_bounds), isLeaf(true), depth(_depth) {}

        bool contains(Object3D* object) const {
            for (const auto& obj : objects) {
                if (obj == object) return true;
            }
            return false;
        }
    };

    std::unique_ptr<OctreeNode> root;
    int maxDepth;
    int maxObjectsPerNode;
    std::vector<std::pair<Object3D*, Object3D*>> potentialCollisions;
    std::unordered_map<Object3D*, std::vector<OctreeNode*>> objectToNodes;

public:
    Octree(const AABB& worldBounds, int _maxDepth = 8, int _maxObjectsPerNode = 10)
        : maxDepth(_maxDepth), maxObjectsPerNode(_maxObjectsPerNode) {
        root = std::make_unique<OctreeNode>(worldBounds, 0);
    }

    void addObject(Object3D* object) {
        objectToNodes[object] = std::vector<OctreeNode*>();
        insertObject(root.get(), object);
    }

    void removeObject(Object3D* object) {
        // Remove object from all nodes it belongs to
        if (objectToNodes.find(object) != objectToNodes.end()) {
            for (OctreeNode* node : objectToNodes[object]) {
                auto& objects = node->objects;
                objects.erase(std::remove(objects.begin(), objects.end(), object), objects.end());
            }
            objectToNodes.erase(object);
        }
    }

    void updateObject(Object3D* object) {
        removeObject(object);
        addObject(object);
    }

    void clear() {
        root->objects.clear();
        root->children.fill(nullptr);
        root->isLeaf = true;
        objectToNodes.clear();
    }

    void findPotentialCollisions() {
        potentialCollisions.clear();
        std::unordered_set<std::pair<Object3D*, Object3D*>, PairHash> uniquePairs;
        findPotentialCollisionsRecursive(root.get(), uniquePairs);
    }

    const std::vector<std::pair<Object3D*, Object3D*>>& getPotentialCollisions() const {
        return potentialCollisions;
    }

private:
    void insertObject(OctreeNode* node, Object3D* object) {
        // Check if object's AABB intersects with this node
        if (!aabbIntersectsAABB(object->getAABB(), node->bounds)) {
            return;
        }

        // If node is a leaf and has space or has reached max depth
        if (node->isLeaf && (node->objects.size() < maxObjectsPerNode || node->depth >= maxDepth)) {
            node->objects.push_back(object);
            objectToNodes[object].push_back(node);
            return;
        }

        // If node is a leaf but needs to be split
        if (node->isLeaf) {
            splitNode(node);
        }

        // Insert into appropriate children
        for (auto& child : node->children) {
            if (child) {
                insertObject(child.get(), object);
            }
        }

        // Also insert into this node if it's not a leaf (for overlapping objects)
        if (!node->isLeaf) {
            node->objects.push_back(object);
            objectToNodes[object].push_back(node);
        }
    }

    void splitNode(OctreeNode* node) {
        if (!node->isLeaf) return;

        Vector3 center = (node->bounds.min + node->bounds.max) * 0.5f;

        // Create 8 children nodes
        for (int i = 0; i < 8; ++i) {
            Vector3 min, max;

            min.x = (i & 1) ? center.x : node->bounds.min.x;
            min.y = (i & 2) ? center.y : node->bounds.min.y;
            min.z = (i & 4) ? center.z : node->bounds.min.z;

            max.x = (i & 1) ? node->bounds.max.x : center.x;
            max.y = (i & 2) ? node->bounds.max.y : center.y;
            max.z = (i & 4) ? node->bounds.max.z : center.z;

            AABB childBounds(min, max);
            node->children[i] = std::make_unique<OctreeNode>(childBounds, node->depth + 1);
        }

        // Redistribute objects to children
        std::vector<Object3D*> nodeObjects;
        nodeObjects.swap(node->objects);

        for (Object3D* obj : nodeObjects) {
            // Remove this node from object's node list
            auto& nodes = objectToNodes[obj];
            nodes.erase(std::remove(nodes.begin(), nodes.end(), node), nodes.end());

            // Reinsert object to appropriate children
            for (auto& child : node->children) {
                if (aabbIntersectsAABB(obj->getAABB(), child->bounds)) {
                    child->objects.push_back(obj);
                    objectToNodes[obj].push_back(child.get());
                }
            }
        }

        node->isLeaf = false;
    }

    bool aabbIntersectsAABB(const AABB& a, const AABB& b) {
        return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
            (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
            (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }

    void findPotentialCollisionsRecursive(OctreeNode* node,
        std::unordered_set<std::pair<Object3D*, Object3D*>, PairHash>& uniquePairs) {
        // Check for collisions between objects in this node
        for (size_t i = 0; i < node->objects.size(); ++i) {
            for (size_t j = i + 1; j < node->objects.size(); ++j) {
                Object3D* obj1 = node->objects[i];
                Object3D* obj2 = node->objects[j];

                // Create a consistent pair (smaller pointer first to avoid duplicates)
                auto pair = obj1 < obj2 ?
                    std::make_pair(obj1, obj2) :
                    std::make_pair(obj2, obj1);

                // Only add if not already added
                if (uniquePairs.find(pair) == uniquePairs.end()) {
                    uniquePairs.insert(pair);

                    // Perform AABB overlap test
                    if (aabbIntersectsAABB(obj1->getAABB(), obj2->getAABB())) {
                        potentialCollisions.push_back(pair);
                    }
                }
            }
        }

        // Recursively check children
        if (!node->isLeaf) {
            for (const auto& child : node->children) {
                if (child) {
                    findPotentialCollisionsRecursive(child.get(), uniquePairs);
                }
            }
        }
    }

    struct PairHash {
        size_t operator()(const std::pair<Object3D*, Object3D*>& p) const {
            auto h1 = std::hash<Object3D*>()(p.first);
            auto h2 = std::hash<Object3D*>()(p.second);
            return h1 ^ (h2 << 1);
        }
    };
};