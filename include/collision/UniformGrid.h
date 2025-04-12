#pragma once

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include "Object3D.h"
#include "Vector3.h"
#include "AABB.h"

class UniformGrid {
private:
    struct GridCell {
        int x, y, z;

        GridCell(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}

        bool operator==(const GridCell& other) const {
            return x == other.x && y == other.y && z == other.z;
        }

        struct Hash {
            size_t operator()(const GridCell& cell) const {
                // Hash function for the grid cell
                size_t hash = 17;
                hash = hash * 31 + std::hash<int>()(cell.x);
                hash = hash * 31 + std::hash<int>()(cell.y);
                hash = hash * 31 + std::hash<int>()(cell.z);
                return hash;
            }
        };
    };

    float cellSize;
    Vector3 worldMin;
    Vector3 worldMax;
    std::unordered_map<GridCell, std::vector<Object3D*>, GridCell::Hash> cells;
    std::unordered_map<Object3D*, std::vector<GridCell>> objectCells;
    std::vector<std::pair<Object3D*, Object3D*>> potentialCollisions;

public:
    UniformGrid(float _cellSize, const Vector3& _worldMin, const Vector3& _worldMax)
        : cellSize(_cellSize), worldMin(_worldMin), worldMax(_worldMax) {
    }

    void addObject(Object3D* object) {
        const AABB& aabb = object->getAABB();
        std::vector<GridCell> occupiedCells = getCellsForAABB(aabb);

        for (const auto& cell : occupiedCells) {
            cells[cell].push_back(object);
        }

        objectCells[object] = occupiedCells;
    }

    void removeObject(Object3D* object) {
        if (objectCells.find(object) == objectCells.end()) return;

        // Remove object from all cells it occupies
        for (const auto& cell : objectCells[object]) {
            auto& cellObjects = cells[cell];
            auto it = std::find(cellObjects.begin(), cellObjects.end(), object);
            if (it != cellObjects.end()) {
                cellObjects.erase(it);
            }

            // If cell is empty, remove it
            if (cellObjects.empty()) {
                cells.erase(cell);
            }
        }

        objectCells.erase(object);
    }

    void updateObject(Object3D* object) {
        removeObject(object);
        addObject(object);
    }

    std::vector<GridCell> getCellsForAABB(const AABB& aabb) {
        std::vector<GridCell> result;

        // Calculate grid cell indices for min and max corners
        int minCellX = static_cast<int>((aabb.min.x - worldMin.x) / cellSize);
        int minCellY = static_cast<int>((aabb.min.y - worldMin.y) / cellSize);
        int minCellZ = static_cast<int>((aabb.min.z - worldMin.z) / cellSize);

        int maxCellX = static_cast<int>((aabb.max.x - worldMin.x) / cellSize);
        int maxCellY = static_cast<int>((aabb.max.y - worldMin.y) / cellSize);
        int maxCellZ = static_cast<int>((aabb.max.z - worldMin.z) / cellSize);

        // Iterate through all cells that this AABB occupies
        for (int x = minCellX; x <= maxCellX; ++x) {
            for (int y = minCellY; y <= maxCellY; ++y) {
                for (int z = minCellZ; z <= maxCellZ; ++z) {
                    result.emplace_back(x, y, z);
                }
            }
        }

        return result;
    }

    void findPotentialCollisions() {
        potentialCollisions.clear();
        std::unordered_set<std::pair<Object3D*, Object3D*>, PairHash> uniquePairs;

        // For each non-empty cell
        for (const auto& cell : cells) {
            const auto& cellObjects = cell.second;

            // Check for potential collisions between objects in the same cell
            for (size_t i = 0; i < cellObjects.size(); ++i) {
                for (size_t j = i + 1; j < cellObjects.size(); ++j) {
                    Object3D* obj1 = cellObjects[i];
                    Object3D* obj2 = cellObjects[j];

                    // Create a consistent pair (smaller pointer first to avoid duplicates)
                    auto pair = obj1 < obj2 ?
                        std::make_pair(obj1, obj2) :
                        std::make_pair(obj2, obj1);

                    // Only add if not already added
                    if (uniquePairs.find(pair) == uniquePairs.end()) {
                        uniquePairs.insert(pair);

                        // Perform AABB overlap test
                        if (checkAABBOverlap(obj1->getAABB(), obj2->getAABB())) {
                            potentialCollisions.push_back(pair);
                        }
                    }
                }
            }
        }
    }

    bool checkAABBOverlap(const AABB& a, const AABB& b) {
        return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
            (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
            (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }

    const std::vector<std::pair<Object3D*, Object3D*>>& getPotentialCollisions() const {
        return potentialCollisions;
    }

    struct PairHash {
        size_t operator()(const std::pair<Object3D*, Object3D*>& p) const {
            auto h1 = std::hash<Object3D*>()(p.first);
            auto h2 = std::hash<Object3D*>()(p.second);
            return h1 ^ (h2 << 1);
        }
    };
};