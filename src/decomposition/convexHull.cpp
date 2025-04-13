#include "ConvexHull.h"
#include "../math/Vector3.h"
#include "../core/Object3D.h"

Vector3 ConvexHull::getSupportPoint(const Vecotr3& direction) const {
    if (vertices.empty()) return Vector3(0, 0, 0);
    
    Vector3 furthestPoint = vertices[0];
    float maxDistance = Vector3::dot(direction, vertices[0]);
    
    for (size_t i = 1; i < vertices.size(); i++) {
        float distance = Vector3::dot(direction, vertices[i]);
        if (distance > maxDistance) {
            maxDistance = distance;
            furthestPoint = vertices[i];
        }
    }
    return furthestPoint;
}