// Object3D.cpp
#include "Object3D.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <limits>
#include <iostream> // std::cerr를 위한 포함

#include "VHACD.h"

// CollisionInfo 구현
CollisionInfo::CollisionInfo() : otherObject(nullptr), penetrationDepth(0.0f) {}

CollisionInfo::CollisionInfo(Object3D* other, const Vector3& point, const Vector3& normal, float depth)
    : otherObject(other), contactPoint(point), contactNormal(normal), penetrationDepth(depth) {}

// Object3D 구현
Object3D::Object3D(const std::string& _name)
    : name(_name),
    position(Vector3(0, 0, 0)),
    rotation(Quaternion::identity()), // Quaternion으로 초기화
    scale(Vector3(1, 1, 1)),
    transformDirty(true),
    aabbDirty(true),
    isInCollision(false),
    isConvexDecomposed(false) {

    // 기본 로컬 AABB(원점 중심의 단위 큐브)
    localAABB.min = Vector3(-0.5f, -0.5f, -0.5f);
    localAABB.max = Vector3(0.5f, 0.5f, 0.5f);

    updateTransformMatrix();
    updateWorldAABB();
}

// 위치 관련 메소드
const Vector3& Object3D::getPosition() const {
    return position;
}

void Object3D::setPosition(const Vector3& pos) {
    position = pos;
    transformDirty = true; // 변환 행렬 갱신 필요
    aabbDirty = true;      // AABB 갱신 필요
}

void Object3D::translate(const Vector3& offset) {
    position += offset;
    transformDirty = true;
    aabbDirty = true;
}

// 회전 관련 메소드
const Quaternion& Object3D::getRotation() const {
    return rotation;
}

void Object3D::setRotation(const Quaternion& rot) {
    rotation = rot;
    transformDirty = true;
    aabbDirty = true;
}

void Object3D::rotate(const Quaternion& rot) {
    rotation = rotation * rot; // 쿼터니언 곱으로 회전 누적
    transformDirty = true;
    aabbDirty = true;
}

void Object3D::rotateAxis(const Vector3& axis, float angleRadians) {
    Quaternion q = Quaternion::fromAxisAngle(axis, angleRadians);
    rotate(q);
}

// 스케일 관련 메소드
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

// 변환 행렬 연산
const Matrix3x3& Object3D::getTransformMatrix() {
    if (transformDirty) {
        updateTransformMatrix();
    }
    return transformMatrix;
}

void Object3D::updateTransformMatrix() {
    // 쿼터니언에서 회전 행렬 생성
    Matrix3x3 rotMat = rotation.toRotationMatrix();

    // 스케일 적용
    // 수정됨: operator[] 대신 Matrix3x3의 operator() 사용
    rotMat(0, 0) *= scale.x;
    rotMat(1, 0) *= scale.x;
    rotMat(2, 0) *= scale.x;

    rotMat(0, 1) *= scale.y;
    rotMat(1, 1) *= scale.y;
    rotMat(2, 1) *= scale.y;

    rotMat(0, 2) *= scale.z;
    rotMat(1, 2) *= scale.z;
    rotMat(2, 2) *= scale.z;

    // 주의: Matrix3x3는 3x3 행렬이므로 이동 요소를 직접 저장할 수 없음
    // 이상적으로는 Matrix4x4를 사용하거나 이동 벡터를 별도로 관리해야 함
    // 현재는 모든 이동 연산에서 position을 직접 사용해야 함

    transformMatrix = rotMat;
    transformDirty = false;
}

// AABB 연산
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

    // 변환된 8개의 모서리 점으로 초기화
    Vector3 corners[8];
    corners[0] = transformPoint(Vector3(localAABB.min.x, localAABB.min.y, localAABB.min.z));
    corners[1] = transformPoint(Vector3(localAABB.max.x, localAABB.min.y, localAABB.min.z));
    corners[2] = transformPoint(Vector3(localAABB.min.x, localAABB.max.y, localAABB.min.z));
    corners[3] = transformPoint(Vector3(localAABB.max.x, localAABB.max.y, localAABB.min.z));
    corners[4] = transformPoint(Vector3(localAABB.min.x, localAABB.min.y, localAABB.max.z));
    corners[5] = transformPoint(Vector3(localAABB.max.x, localAABB.min.y, localAABB.max.z));
    corners[6] = transformPoint(Vector3(localAABB.min.x, localAABB.max.y, localAABB.max.z));
    corners[7] = transformPoint(Vector3(localAABB.max.x, localAABB.max.y, localAABB.max.z));

    // 최소 및 최대 점 찾기
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
    // 수정됨: Matrix3x3의 operator() 사용
    // 회전과 스케일 적용 후 위치 더하기
    result.x = transformMatrix(0, 0) * point.x + transformMatrix(0, 1) * point.y +
        transformMatrix(0, 2) * point.z + position.x;
    result.y = transformMatrix(1, 0) * point.x + transformMatrix(1, 1) * point.y +
        transformMatrix(1, 2) * point.z + position.y;
    result.z = transformMatrix(2, 0) * point.x + transformMatrix(2, 1) * point.y +
        transformMatrix(2, 2) * point.z + position.z;
    return result;
}

Vector3 Object3D::transformDirection(const Vector3& dir) {
    if (transformDirty) {
        updateTransformMatrix();
    }

    Vector3 result;
    // 방향 벡터에는 위치 변환 적용하지 않음 (회전과 스케일만 적용)
    result.x = transformMatrix(0, 0) * dir.x + transformMatrix(0, 1) * dir.y + transformMatrix(0, 2) * dir.z;
    result.y = transformMatrix(1, 0) * dir.x + transformMatrix(1, 1) * dir.y + transformMatrix(1, 2) * dir.z;
    result.z = transformMatrix(2, 0) * dir.x + transformMatrix(2, 1) * dir.y + transformMatrix(2, 2) * dir.z;
    return result;
}

// 충돌 관리
bool Object3D::isColliding() const {
    return isInCollision;
}

const std::vector<CollisionInfo>& Object3D::getCollisions() const {
    return collisions;
}

void Object3D::addCollision(const CollisionInfo& collision) {
    // 이미 이 객체와 충돌 중인지 확인
    for (const auto& existing : collisions) {
        if (existing.otherObject == collision.otherObject) {
            return; // 이미 이 객체와 충돌 중
        }
    }

    // 새 충돌 추가
    collisions.push_back(collision);

    // 이것이 첫 번째 충돌이면 onCollisionEnter 트리거
    if (!isInCollision) {
        isInCollision = true;
        if (onCollisionEnter) {
            onCollisionEnter(collision);
        }
    }
    else {
        // 그렇지 않으면 onCollisionStay 트리거
        if (onCollisionStay) {
            onCollisionStay(collision);
        }
    }
}

void Object3D::removeCollision(Object3D* other) {
    auto it = std::find_if(collisions.begin(), collisions.end(),
        [other](const CollisionInfo& info) { return info.otherObject == other; });

    if (it != collisions.end()) {
        // onCollisionExit 콜백 트리거
        if (onCollisionExit) {
            onCollisionExit(*it);
        }

        // 충돌 제거
        collisions.erase(it);

        // 충돌 상태 업데이트
        isInCollision = !collisions.empty();
    }
}

void Object3D::clearCollisions() {
    // 모든 활성 충돌에 대해 onCollisionExit 트리거
    if (onCollisionExit) {
        for (const auto& collision : collisions) {
            onCollisionExit(collision);
        }
    }

    collisions.clear();
    isInCollision = false;
}

// 충돌 이벤트 콜백
void Object3D::setOnCollisionEnter(const CollisionCallback& callback) {
    onCollisionEnter = callback;
}

void Object3D::setOnCollisionStay(const CollisionCallback& callback) {
    onCollisionStay = callback;
}

void Object3D::setOnCollisionExit(const CollisionCallback& callback) {
    onCollisionExit = callback;
}

// 메시 데이터 연산
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

// 파일 로드 연산
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
            Vector3 normal = edge1.cross(edge2).normalized();
            
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
    if (this->isConvexDecomposed) {
        Vector3 furthestPoint(0, 0, 0);
        float maxDistance = -std::numeric_limits<float>::max();
        
        // 회전된 방향을 구하기 위해, rotation.inverse() 및 rotate()를 사용 (Quaternion 타입에 해당 메서드가 정의되어 있어야 함)
        Quaternion invRot = this->rotation.inverse();
        Vector3 localDir = invRot.rotate(direction);
        
        for (const auto& hull : this->convexHulls) {
            Vector3 localSupport = hull.getSupportPoint(localDir);
            Vector3 worldSupport = this->rotation.rotate(localSupport) + this->position;
            float distance = direction.dot(worldSupport);
            if (distance > maxDistance) {
                maxDistance = distance;
                furthestPoint = worldSupport;
            }
        }
        
        return furthestPoint;
    } else {
        if (this->vertices.empty())
            return this->position;
        
        Vector3 furthestPoint = this->position;
        float maxDistance = -std::numeric_limits<float>::max();
        Quaternion invRot = this->rotation.inverse();
        Vector3 localDir = invRot.rotate(direction);
        for (const auto& vertex : this->vertices) {
            Vector3 worldVertex = this->rotation.rotate(vertex) + this->position;
            float distance = direction.dot(worldVertex);
            if (distance > maxDistance) {
                maxDistance = distance;
                furthestPoint = worldVertex;
            }
        }
        return furthestPoint;
    }
}

// 접근자(Accessors)
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
    if (this->vertices.empty() || this->indices.empty()) {
        std::cerr << "Cannot compute convex decomposition: No mesh data loaded." << std::endl;
        return false;
    }
    
    this->convexHulls = ConvexDecomposition::ComputeConvexDecomposition(this->vertices, this->indices, params);
    this->isConvexDecomposed = !this->convexHulls.empty();
    
    if (this->isConvexDecomposed) {
        std::vector<Vector3> allVertices;
        for (const auto& hull : this->convexHulls) {
            allVertices.insert(allVertices.end(), hull.vertices.begin(), hull.vertices.end());
        }
        
        if (!allVertices.empty()) {
            AABB combinedBounds(allVertices);
            setLocalAABB(combinedBounds);
        }
    }
    
    return this->isConvexDecomposed;
}

bool Object3D::loadConvexDecomposition(const std::string& filepath) {
    this->convexHulls = ConvexDecomposition::LoadConvexHulls(filepath);
    this->isConvexDecomposed = !this->convexHulls.empty();
    
    if (this->isConvexDecomposed) {
        std::vector<Vector3> allVertices;
        for (const auto& hull : this->convexHulls) {
            allVertices.insert(allVertices.end(), hull.vertices.begin(), hull.vertices.end());
        }
        
        if (!allVertices.empty()) {
            AABB combinedBounds(allVertices);
            setLocalAABB(combinedBounds);
        }
    }
    
    return this->isConvexDecomposed;
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