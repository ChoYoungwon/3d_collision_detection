
// Object3D.cpp
#include "Object3D.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <limits>

#include "VHACD.h"

// CollisionInfo 구현
CollisionInfo::CollisionInfo() : otherObject(nullptr), penetrationDepth(0.0f) {}

CollisionInfo::CollisionInfo(Object3D* other, const Vector3& point, const Vector3& normal, float depth)
    : otherObject(other), contactPoint(point), contactNormal(normal), penetrationDepth(depth) {}

// ConvexHull 구현
Vector3 ConvexHull::getSupportPoint(const Vector3& direction) const {
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

// Object3D 구현
Object3D::Object3D(const std::string& _name)
    : name(_name),
    position(Vector3(0, 0, 0)),
    rotation(Quaternion::identity()),
    scale(Vector3(1, 1, 1)),
    transformDirty(true),
    aabbDirty(true),
    isInCollision(false),
    isConvexDecomposed(false) {

    // Default local AABB (unit cube centered at origin)
    localAABB.min = Vector3(-0.5f, -0.5f, -0.5f);
    localAABB.max = Vector3(0.5f, 0.5f, 0.5f);

    updateTransformMatrix();
    updateWorldAABB();
}

// Position methods
const Vector3& Object3D::getPosition() const {
    return position;
}

void Object3D::setPosition(const Vector3& pos) {
    position = pos;
    transformDirty = true;
    aabbDirty = true;
}

void Object3D::translate(const Vector3& offset) {
    position += offset;
    transformDirty = true;
    aabbDirty = true;
}

// Rotation methods
const Quaternion& Object3D::getRotation() const {
    return rotation;
}

void Object3D::setRotation(const Quaternion& rot) {
    rotation = rot;
    transformDirty = true;
    aabbDirty = true;
}

void Object3D::rotate(const Quaternion& rot) {
    rotation = rotation * rot;
    transformDirty = true;
    aabbDirty = true;
}

void Object3D::rotateAxis(const Vector3& axis, float angleRadians) {
    Quaternion q = Quaternion::fromAxisAngle(axis, angleRadians);
    rotate(q);
}

// Scale methods
const Vector3& Object3D::getScale() const {
    return scale;
}

void Object3D::setScale(const Vector3& s) {
    scale = s;
    transformDirty = true;
    aabbDirty = true;
}

void Object3D::setScale(float uniformScale) {
    scale = Vector3(uniformScale, uniformScale, uniformScale);
    transformDirty = true;
    aabbDirty = true;
}

// Transform matrix operations
const Matrix3x3& Object3D::getTransformMatrix() {
    if (transformDirty) {
        updateTransformMatrix();
    }
    return transformMatrix;
}

void Object3D::updateTransformMatrix() {
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
void Object3D::setLocalAABB(const AABB& aabb) {
    localAABB = aabb;
    aabbDirty = true;
}

const AABB& Object3D::getLocalAABB() const {
    return localAABB;
}

const AABB& Object3D::getAABB() {
    if (aabbDirty) {
        updateWorldAABB();
    }
    return worldAABB;
}

void Object3D::updateWorldAABB() {
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

Vector3 Object3D::transformPoint(const Vector3& point) {
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

Vector3 Object3D::transformDirection(const Vector3& dir) {
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
bool Object3D::isColliding() const {
    return isInCollision;
}

const std::vector<CollisionInfo>& Object3D::getCollisions() const {
    return collisions;
}

void Object3D::addCollision(const CollisionInfo& collision) {
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

void Object3D::removeCollision(Object3D* other) {
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

void Object3D::clearCollisions() {
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
void Object3D::setOnCollisionEnter(const CollisionCallback& callback) {
    onCollisionEnter = callback;
}

void Object3D::setOnCollisionStay(const CollisionCallback& callback) {
    onCollisionStay = callback;
}

void Object3D::setOnCollisionExit(const CollisionCallback& callback) {
    onCollisionExit = callback;
}

// Mesh data operations
void Object3D::setMeshData(const std::vector<Vector3>& verts, 
                           const std::vector<Vector3>& norms,
                           const std::vector<int>& inds) {
    vertices = verts;
    normals = norms;
    indices = inds;
    
    // 정점 데이터에서 AABB 자동 계산
    if (!vertices.empty()) {
        AABB meshBounds(vertices);
        setLocalAABB(meshBounds);
    }
}

// File loading operations
bool Object3D::loadFromObjFile(const std::string& filepath) {
    std::vector<Vector3> loadedVertices;
    std::vector<Vector3> loadedNormals;
    std::vector<int> loadedIndices;
    
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return false;
    }
    
    std::vector<Vector3> tempVertices;
    std::vector<Vector3> tempNormals;
    
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        
        if (prefix == "v") {  // 정점
            float x, y, z;
            iss >> x >> y >> z;
            tempVertices.push_back(Vector3(x, y, z));
        }
        else if (prefix == "vn") {  // 법선
            float x, y, z;
            iss >> x >> y >> z;
            tempNormals.push_back(Vector3(x, y, z));
        }
        else if (prefix == "f") {  // 면
            std::string v1, v2, v3;
            iss >> v1 >> v2 >> v3;
            
            // 인덱스 파싱 (v/vt/vn 형식 처리)
            auto parseIndex = [](const std::string& str) -> int {
                size_t pos = str.find('/');
                if (pos == std::string::npos) {
                    return std::stoi(str) - 1;  // OBJ는 1부터 인덱싱
                }
                return std::stoi(str.substr(0, pos)) - 1;
            };
            
            int idx1 = parseIndex(v1);
            int idx2 = parseIndex(v2);
            int idx3 = parseIndex(v3);
            
            loadedIndices.push_back(idx1);
            loadedIndices.push_back(idx2);
            loadedIndices.push_back(idx3);
            
            // 정점 복사
            loadedVertices.push_back(tempVertices[idx1]);
            loadedVertices.push_back(tempVertices[idx2]);
            loadedVertices.push_back(tempVertices[idx3]);
            
            // 법선이 있으면 복사
            if (!tempNormals.empty()) {
                // 여기서는 간단함을 위해 법선 인덱스가 정점 인덱스와 동일하다고 가정
                // 실제로는 더 복잡한 파싱이 필요할 수 있음
                loadedNormals.push_back(tempNormals[idx1]);
                loadedNormals.push_back(tempNormals[idx2]);
                loadedNormals.push_back(tempNormals[idx3]);
            }
        }
    }
    
    // 법선이 없으면 계산
    if (loadedNormals.empty() && !loadedIndices.empty()) {
        loadedNormals.resize(loadedVertices.size(), Vector3(0, 0, 0));
        
        // 각 삼각형의 법선 계산
        for (size_t i = 0; i < loadedIndices.size(); i += 3) {
            Vector3 v1 = loadedVertices[loadedIndices[i]];
            Vector3 v2 = loadedVertices[loadedIndices[i+1]];
            Vector3 v3 = loadedVertices[loadedIndices[i+2]];
            
            Vector3 edge1 = v2 - v1;
            Vector3 edge2 = v3 - v1;
            Vector3 normal = Vector3::cross(edge1, edge2).normalized();
            
            // 법선 누적 (나중에 정규화)
            loadedNormals[loadedIndices[i]] += normal;
            loadedNormals[loadedIndices[i+1]] += normal;
            loadedNormals[loadedIndices[i+2]] += normal;
        }
        
        // 법선 정규화
        for (auto& normal : loadedNormals) {
            normal.normalize();
        }
    }
    
    // 메시 데이터 설정
    setMeshData(loadedVertices, loadedNormals, loadedIndices);
    return true;
}

bool Object3D::loadVHACDResult(const std::string& filepath) {
    convexHulls.clear();
    isConvexDecomposed = false;
    
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return false;
    }
    
    std::vector<Vector3> tempVertices;
    std::vector<Vector3> tempNormals;
    ConvexHull currentHull;
    
    std::string line;
    bool newObjectStarted = false;
    
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        
        if (prefix == "o" || prefix == "g") {  // 새 객체/그룹
            if (newObjectStarted && !currentHull.vertices.empty()) {
                // 현재 볼록 껍질 저장하고 새 껍질 시작
                convexHulls.push_back(currentHull);
                currentHull = ConvexHull();
            }
            newObjectStarted = true;
        }
        else if (prefix == "v") {  // 정점
            float x, y, z;
            iss >> x >> y >> z;
            tempVertices.push_back(Vector3(x, y, z));
        }
        else if (prefix == "f") {  // 면
            if (tempVertices.empty()) continue;
            
            std::string v1, v2, v3;
            iss >> v1 >> v2 >> v3;
            
            // 인덱스 파싱 (v/vt/vn 형식 처리)
            auto parseIndex = [](const std::string& str) -> int {
                size_t pos = str.find('/');
                if (pos == std::string::npos) {
                    return std::stoi(str) - 1;  // OBJ는 1부터 인덱싱
                }
                return std::stoi(str.substr(0, pos)) - 1;
            };
            
            int idx1 = parseIndex(v1);
            int idx2 = parseIndex(v2);
            int idx3 = parseIndex(v3);
            
            // 볼록 껍질에 정점 추가
            currentHull.vertices.push_back(tempVertices[idx1]);
            currentHull.vertices.push_back(tempVertices[idx2]);
            currentHull.vertices.push_back(tempVertices[idx3]);
            
            // 인덱스 추가 (로컬 인덱스)
            int baseIndex = currentHull.vertices.size() - 3;
            currentHull.indices.push_back(baseIndex);
            currentHull.indices.push_back(baseIndex + 1);
            currentHull.indices.push_back(baseIndex + 2);
        }
    }
    
    // 마지막 볼록 껍질 추가
    if (!currentHull.vertices.empty()) {
        convexHulls.push_back(currentHull);
    }
    
    // AABB 계산
    if (!convexHulls.empty()) {
        std::vector<Vector3> allVertices;
        for (const auto& hull : convexHulls) {
            allVertices.insert(allVertices.end(), hull.vertices.begin(), hull.vertices.end());
        }
        
        if (!allVertices.empty()) {
            AABB combinedBounds(allVertices);
            setLocalAABB(combinedBounds);
        }
        
        isConvexDecomposed = true;
    }
    
    return !convexHulls.empty();
}

Vector3 Object3D::getSupportPoint(const Vector3& direction) const {
    if (isConvexDecomposed) {
        // 모든 볼록 껍질 중에서 가장 먼 점 찾기
        Vector3 furthestPoint = Vector3(0, 0, 0);
        float maxDistance = -std::numeric_limits<float>::max();
        
        for (const auto& hull : convexHulls) {
            // 방향 벡터를 로컬 좌표계로 변환
            Vector3 localDir = rotation.inverse() * direction;
            
            // 로컬 좌표계에서 지원점 찾기
            Vector3 localSupport = hull.getSupportPoint(localDir);
            
            // 월드 좌표계로 변환
            Vector3 worldSupport = rotation * localSupport + position;
            
            float distance = Vector3::dot(direction, worldSupport);
            
            if (distance > maxDistance) {
                maxDistance = distance;
                furthestPoint = worldSupport;
            }
        }
        
        return furthestPoint;
    }
    else {
        // 볼록 분해가 없으면 원본 메시 사용
        if (vertices.empty()) return position;
        
        Vector3 furthestPoint = position;
        float maxDistance = -std::numeric_limits<float>::max();
        
        // 방향 벡터를 로컬 좌표계로 변환
        Vector3 localDir = rotation.inverse() * direction;
        
        for (const auto& vertex : vertices) {
            // 정점을 월드 좌표계로 변환
            Vector3 worldVertex = rotation * vertex + position;
            float distance = Vector3::dot(direction, worldVertex);
            
            if (distance > maxDistance) {
                maxDistance = distance;
                furthestPoint = worldVertex;
            }
        }
        
        return furthestPoint;
    }
}

// Accessors
bool Object3D::isDecomposed() const { 
    return isConvexDecomposed; 
}

const std::vector<ConvexHull>& Object3D::getConvexHulls() const { 
    return convexHulls; 
}

const std::vector<Vector3>& Object3D::getVertices() const { 
    return vertices; 
}

const std::vector<Vector3>& Object3D::getNormals() const { 
    return normals; 
}

const std::vector<int>& Object3D::getIndices() const { 
    return indices; 
}

const std::string& Object3D::getName() const {
    return name;
}

void Object3D::setName(const std::string& _name) {
    name = _name;
}

void Object3D::update() {
    if (transformDirty) {
        updateTransformMatrix();
    }

    if (aabbDirty) {
        updateWorldAABB();
    }
}

bool Object3D::computeConvexDecomposition(const VHACDParameters& params) {
    if (vertices.empty() || indices.empty()) {
        std::cerr << "Cannot compute convex decomposition: No mesh data loaded." << std::endl;
        return false;
    }
    
    // ConvexDecomposition 클래스를 통해 V-HACD API 호출
    convexHulls = ConvexDecomposition::ComputeConvexDecomposition(vertices, indices, params);
    isConvexDecomposed = !convexHulls.empty();
    
    if (isConvexDecomposed) {
        // 모든 볼록 껍질의 정점을 합쳐서 전체 AABB 계산
        std::vector<Vector3> allVertices;
        for (const auto& hull : convexHulls) {
            allVertices.insert(allVertices.end(), hull.vertices.begin(), hull.vertices.end());
        }
        
        if (!allVertices.empty()) {
            AABB combinedBounds(allVertices);
            setLocalAABB(combinedBounds);
        }
    }
    
    return isConvexDecomposed;
}

bool Object3D::loadConvexDecomposition(const std::string& filepath) {
    // 볼록 분해 결과 로드
    convexHulls = ConvexDecomposition::LoadConvexHulls(filepath);
    isConvexDecomposed = !convexHulls.empty();
    
    if (isConvexDecomposed) {
        // 모든 볼록 껍질의 정점을 합쳐서 전체 AABB 계산
        std::vector<Vector3> allVertices;
        for (const auto& hull : convexHulls) {
            allVertices.insert(allVertices.end(), hull.vertices.begin(), hull.vertices.end());
        }
        
        if (!allVertices.empty()) {
            AABB combinedBounds(allVertices);
            setLocalAABB(combinedBounds);
        }
    }
    
    return isConvexDecomposed;
}

void Object3D::setConvexHulls(const std::vector<ConvexHull>& hulls) {
    convexHulls = hulls;
    isConvexDecomposed = !convexHulls.empty();
    
    if (isConvexDecomposed) {
        // 모든 볼록 껍질의 정점을 합쳐서 전체 AABB 계산
        std::vector<Vector3> allVertices;
        for (const auto& hull : convexHulls) {
            allVertices.insert(allVertices.end(), hull.vertices.begin(), hull.vertices.end());
        }
        
        if (!allVertices.empty()) {
            AABB combinedBounds(allVertices);
            setLocalAABB(combinedBounds);
        }
    }
}



