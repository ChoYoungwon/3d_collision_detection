#include "AABB.h"

void AABB::computeFromPoints(const std::vector<Vector3>& points) {
    if (points.empty()) {
        return;
    }

    min = points[0];
    max = points[0];

    for (size_t i = 1; i < points.size(); ++i) {
        min.x = std::min(min.x, points[i].x);
        min.y = std::min(min.y, points[i].y);
        min.z = std::min(min.z, points[i].z);

        max.x = std::max(max.x, points[i].x);
        max.y = std::max(max.y, points[i].y);
        max.z = std::max(max.z, points[i].z);
    }
}