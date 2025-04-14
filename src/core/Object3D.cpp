#include "Object3D.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <limits>
#include <iostream>

// CollisionInfo 구현
CollisionInfo::CollisionInfo() 
    : otherObject(nullptr), contactPoint(Vector3()), contactNormal(Vector3()), penetrationDepth(0.0f) {}

CollisionInfo::CollisionInfo(Object3D* other, const Vector3& point, const Vector3& normal, float depth)
    : otherObject(other), contactPoint(point), contactNormal(normal), penetrationDepth(depth) {}

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

    // 기본 로컬 AABB(원점 중심의 단위 큐브)
    localAABB.min = Vector3(-0.5f, -0.5f, -0.5f);
    localAABB.max = Vector3(0.5f, 0.5f, 0.5f);

    updateTransformMatrix();
    updateWorldAABB();
}

// 현재 월드 위치를 반환
const Vector3& Object3D::getPosition() const {
    return position;
}

// 객체의 월드 위치를 설정, 변환 행렬 갱신
void Object3D::setPosition(const Vector3& pos) {
    position = pos;
    transformDirty = true;
    aabbDirty = true;
}

// 현재 위치에서 특정 오프셋만큼 이동
void Object3D::translate(const Vector3& offset) {
    position += offset;
    transformDirty = true;
    aabbDirty = true;
}

// 현재 회전 쿼터니언을 반환
const Quaternion& Object3D::getRotation() const {
    return rotation;
}

// 객체의 회전을 직접 설정
void Object3D::setRotation(const Quaternion& rot) {
    rotation = rot;
    transformDirty = true;
    aabbDirty = true;
}

// 현재 회전에 추가 회전을 적용(쿼터니언 곱)
void Object3D::rotate(const Quaternion& rot) {
    rotation = rotation * rot; // 쿼터니언 곱으로 회전 누적
    transformDirty = true;
    aabbDirty = true;
}

// 특정 축을 중심으로 회전 적용
void Object3D::rotateAxis(const Vector3& axis, float angleRadians) {
    Quaternion q = Quaternion::fromAxisAngle(axis, angleRadians);
    rotate(q);
}

// 현재 스케일 반환 
const Vector3& Object3D::getScale() const {
    return scale;
}

// X, Y, Z 축별 다른 스케일을 설정 
void Object3D::setScale(const Vector3& s) {
    scale = s;
    transformDirty = true;
    aabbDirty = true;
}

// 모든 축에 동일한 스케일을 설정
void Object3D::setScale(float uniformScale) {
    scale = Vector3(uniformScale, uniformScale, uniformScale);
    transformDirty = true;
    aabbDirty = true;
}

// 변환 행렬을 반환 
const Matrix3x3& Object3D::getTransformMatrix() {
    if (transformDirty) {
        updateTransformMatrix();
    }
    return transformMatrix;
}

// 현재 위치, 회전, 스케일을 기반으로 변환 행렬 계산
void Object3D::updateTransformMatrix() {
    // 쿼터니언에서 회전 행렬 생성
    Matrix3x3 rotMat = rotation.toRotationMatrix();

    // 스케일 적용
    rotMat(0, 0) *= scale.x;
    rotMat(1, 0) *= scale.x;
    rotMat(2, 0) *= scale.x;

    rotMat(0, 1) *= scale.y;
    rotMat(1, 1) *= scale.y;
    rotMat(2, 1) *= scale.y;

    rotMat(0, 2) *= scale.z;
    rotMat(1, 2) *= scale.z;
    rotMat(2, 2) *= scale.z;

    transformMatrix = rotMat;
    transformDirty = false;
}

// 객체의 로컬 AABB설정
void Object3D::setLocalAABB(const AABB& aabb) {
    localAABB = aabb;
    aabbDirty = true;
}

// 로컬 AABB 반환
const AABB& Object3D::getLocalAABB() const {
    return localAABB;
}

// 월드 좌표계 AABB 반환 
const AABB& Object3D::getAABB() {
    if (aabbDirty) {
        updateWorldAABB();
    }
    return worldAABB;
}

// 로컬 AABB에 변환 적용하여 월드 AABB계산 
void Object3D::updateWorldAABB() {
    if (transformDirty) {
        updateTransformMatrix();
    }

    // 로컬 AABB의 8개 모서리 점을 월드 좌표계로 변환
    Vector3 corners[8];
    corners[0] = transformPoint(Vector3(localAABB.min.x, localAABB.min.y, localAABB.min.z));
    corners[1] = transformPoint(Vector3(localAABB.max.x, localAABB.min.y, localAABB.min.z));
    corners[2] = transformPoint(Vector3(localAABB.min.x, localAABB.max.y, localAABB.min.z));
    corners[3] = transformPoint(Vector3(localAABB.max.x, localAABB.max.y, localAABB.min.z));
    corners[4] = transformPoint(Vector3(localAABB.min.x, localAABB.min.y, localAABB.max.z));
    corners[5] = transformPoint(Vector3(localAABB.max.x, localAABB.min.y, localAABB.max.z));
    corners[6] = transformPoint(Vector3(localAABB.min.x, localAABB.max.y, localAABB.max.z));
    corners[7] = transformPoint(Vector3(localAABB.max.x, localAABB.max.y, localAABB.max.z));

    // 변환된 점들로 새 AABB 계산
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

// 로컬 좌표를 월드 좌표로 변환
Vector3 Object3D::transformPoint(const Vector3& point) {
    if (transformDirty) {
        updateTransformMatrix();
    }

    // 회전 및 스케일 적용 후 위치 더하기
    Vector3 result;
    result.x = transformMatrix(0, 0) * point.x + transformMatrix(0, 1) * point.y +
              transformMatrix(0, 2) * point.z + position.x;
    result.y = transformMatrix(1, 0) * point.x + transformMatrix(1, 1) * point.y +
              transformMatrix(1, 2) * point.z + position.y;
    result.z = transformMatrix(2, 0) * point.x + transformMatrix(2, 1) * point.y +
              transformMatrix(2, 2) * point.z + position.z;
    return result;
}

// 로컬 방향 벡터를 월드 방향으로 변환
Vector3 Object3D::transformDirection(const Vector3& dir) {
    if (transformDirty) {
        updateTransformMatrix();
    }

    // 방향 벡터에는 위치 변환을 적용하지 않음 (회전과 스케일만 적용)
    Vector3 result;
    result.x = transformMatrix(0, 0) * dir.x + transformMatrix(0, 1) * dir.y + transformMatrix(0, 2) * dir.z;
    result.y = transformMatrix(1, 0) * dir.x + transformMatrix(1, 1) * dir.y + transformMatrix(1, 2) * dir.z;
    result.z = transformMatrix(2, 0) * dir.x + transformMatrix(2, 1) * dir.y + transformMatrix(2, 2) * dir.z;
    return result;
}

// 월드 좌표를 로컬 좌표로 변환 
Vector3 Object3D::inverseTransformPoint(const Vector3& worldPoint) {
    if (transformDirty) {
        updateTransformMatrix();
    }

    // 위치 차이 계산
    Vector3 positionDiff = worldPoint - position;

    // 역행렬을 직접 계산하는 대신 쿼터니언 역회전과 역스케일 적용
    Quaternion invRotation = rotation.inverse();
    Vector3 invScale(1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z);

    // 역회전 적용
    Vector3 rotatedBack = invRotation.rotate(positionDiff);

    // 역스케일 적용
    return Vector3(rotatedBack.x * invScale.x, rotatedBack.y * invScale.y, rotatedBack.z * invScale.z);
}

// 월드 방향 벡터를 로컬 방향으로 변환 
Vector3 Object3D::inverseTransformDirection(const Vector3& worldDir) {
    if (transformDirty) {
        updateTransformMatrix();
    }

    // 역회전과 역스케일만 적용 (위치 무시)
    Quaternion invRotation = rotation.inverse();
    Vector3 invScale(1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z);

    // 역회전 적용
    Vector3 rotatedBack = invRotation.rotate(worldDir);

    // 역스케일 적용
    return Vector3(rotatedBack.x * invScale.x, rotatedBack.y * invScale.y, rotatedBack.z * invScale.z);
}

// 충돌 여부
bool Object3D::isColliding() const {
    return isInCollision;
}

// 현재 충돌 정보 목록 반환
const std::vector<CollisionInfo>& Object3D::getCollisions() const {
    return collisions;
}

// 새 충돌 정보 추가 및 콜백 호출
void Object3D::addCollision(const CollisionInfo& collision) {
    // 이미 이 객체와 충돌 중인지 확인
    for (const auto& existing : collisions) {
        if (existing.otherObject == collision.otherObject) {
            return; // 이미 이 객체와 충돌 중
        }
    }

    // 새 충돌 추가
    collisions.push_back(collision);

    // 첫 번째 충돌인 경우 onCollisionEnter 콜백 호출
    if (!isInCollision) {
        isInCollision = true;
        if (onCollisionEnter) {
            onCollisionEnter(collision);
        }
    }
    else {
        // 이미 다른 충돌이 있는 경우 onCollisionStay 콜백 호출
        if (onCollisionStay) {
            onCollisionStay(collision);
        }
    }
}

// 특정 객체와의 충돌 제거 및 콜백 호출
void Object3D::removeCollision(Object3D* other) {
    auto it = std::find_if(collisions.begin(), collisions.end(),
        [other](const CollisionInfo& info) { return info.otherObject == other; });

    if (it != collisions.end()) {
        // onCollisionExit 콜백 호출
        if (onCollisionExit) {
            onCollisionExit(*it);
        }

        // 충돌 제거
        collisions.erase(it);

        // 충돌 상태 업데이트
        isInCollision = !collisions.empty();
    }
}

// 모든 충돌 정보 제거
void Object3D::clearCollisions() {
    // 모든 활성 충돌에 대해 onCollisionExit 콜백 호출
    if (onCollisionExit) {
        for (const auto& collision : collisions) {
            onCollisionExit(collision);
        }
    }

    collisions.clear();
    isInCollision = false;
}

// 충돌 시작 시 호출 콜백 
void Object3D::setOnCollisionEnter(const CollisionCallback& callback) {
    onCollisionEnter = callback;
}

// 충돌 지속 중 호출 콜백
void Object3D::setOnCollisionStay(const CollisionCallback& callback) {
    onCollisionStay = callback;
}

// 호출 종료 시 호출 콜백
void Object3D::setOnCollisionExit(const CollisionCallback& callback) {
    onCollisionExit = callback;
}

// 정점, 법선, 인덱스 데이터 설정, AABB 자동 계산
void Object3D::setMeshData(const std::vector<Vector3>& verts, 
                          const std::vector<Vector3>& norms,
                          const std::vector<int>& inds) {
    vertices = verts;
    normals = norms;
    indices = inds;
    
    // 정점 데이터에서 AABB 자동 계산
    if (!vertices.empty()) {
        AABB meshBounds;
        meshBounds.computeFromPoints(vertices);
        setLocalAABB(meshBounds);
    }
}

// OBJ 파일에서 메시 데이터 로드
bool Object3D::loadFromObjFile(const std::string& filepath) {
    // 정점과 면 정보를 저장할 변수 선언
    std::vector<Vector3> tempVertices;
    std::vector<std::vector<int>> faces;
    ConvexDecomposition convexdecomposition;

    // ConvexDecomposition의 ParseObjFile 메서드를 활용하여 파일 파싱
    if (!ConvexDecomposition::ParseObjFile(filepath, tempVertices, faces)) {
        std::cerr << "Failed to load OBJ file: " << filepath << std::endl;
        return false;
    }

    // 인덱스 배열 및 법선 데이터 준비
    std::vector<int> loadedIndices;
    std::vector<Vector3> loadedNormals(tempVertices.size(), Vector3(0, 0, 0));
    
    // 면 정보를 삼각형 인덱스로 변환
    for (const auto& face : faces) {
        // 면이 3개 이상의 정점으로 구성된 경우 삼각형화
        for (size_t i = 2; i < face.size(); ++i) {
            // 첫 정점과 현재 에지를 이용하여 삼각형 형성
            loadedIndices.push_back(face[0]);
            loadedIndices.push_back(face[i-1]);
            loadedIndices.push_back(face[i]);
            
            // 법선 계산 (없는 경우)
            Vector3 v1 = tempVertices[face[0]];
            Vector3 v2 = tempVertices[face[i-1]];
            Vector3 v3 = tempVertices[face[i]];
            
            Vector3 edge1 = v2 - v1;
            Vector3 edge2 = v3 - v1;
            Vector3 normal = edge1.cross(edge2).normalized();
            
            // 법선 누적
            loadedNormals[face[0]] += normal;
            loadedNormals[face[i-1]] += normal;
            loadedNormals[face[i]] += normal;
        }
    }
    
    // 누적된 법선 정규화
    for (auto& normal : loadedNormals) {
        normal.normalize();
    }
    
    // 메시 데이터 설정
    setMeshData(tempVertices, loadedNormals, loadedIndices);
    return true;
}

// V-HACD를 사용하여 메시의 볼록 분해
bool Object3D::computeConvexDecomposition(
    const std::string& inputObjPath, 
    const std::string& outputObjPath, 
    const VHACDParameters& params
) {
    if(!ConvexDecomposition::RunVHACD(inputObjPath, outputObjPath, params))
        return false;
    return true;
}

// 계산된 볼록 분해 결과 로드
bool Object3D::loadConvexDecomposition(const std::string& filepath) {
    // ConvexDecomposition 클래스를 사용하여 분해된 OBJ 파일 로드
    convexHulls = ConvexDecomposition::LoadConvexHulls(filepath);
    isConvexDecomposed = !convexHulls.empty();
    
    if (isConvexDecomposed) {
        // 모든 볼록 껍질의 정점을 합쳐서 전체 AABB 계산
        std::vector<Vector3> allVertices;
        for (const auto& hull : convexHulls) {
            allVertices.insert(allVertices.end(), hull.vertices.begin(), hull.vertices.end());
        }
        
        if (!allVertices.empty()) {
            AABB combinedBounds;
            combinedBounds.computeFromPoints(allVertices);
            setLocalAABB(combinedBounds);
        }
    }
    
    return isConvexDecomposed;
}

// 기존 볼록 껍질 데이터 설정
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
            AABB combinedBounds;
            combinedBounds.computeFromPoints(allVertices);
            setLocalAABB(combinedBounds);
        }
    }
}

// GJK 알고리즘에 사용되는 특정 방향의 최대 지원점 반환
Vector3 Object3D::getSupportPoint(const Vector3& direction) const {
    if (isConvexDecomposed) {
        Vector3 furthestPoint(0, 0, 0);
        float maxDistance = -std::numeric_limits<float>::max();
        
        // 모든 볼록 껍질에서 지원점 찾기
        for (const auto& hull : convexHulls) {
            // 월드 방향을 로컬 방향으로 변환
            Vector3 localDir = rotation.inverse().rotate(direction);
            
            for (const auto& vertex : hull.vertices) {
                // 각 정점을 월드 좌표계로 변환
                Vector3 worldVertex = rotation.rotate(vertex) + position;
                
                // 방향에 따른 투영 계산
                float distance = direction.dot(worldVertex);
                if (distance > maxDistance) {
                    maxDistance = distance;
                    furthestPoint = worldVertex;
                }
            }
        }
        
        return furthestPoint;
    } 
    else {
        // 볼록 분해가 없는 경우, 메시 정점을 직접 사용
        if (vertices.empty()) {
            return position; // 정점이 없으면 객체 위치 반환
        }
        
        Vector3 furthestPoint = position;
        float maxDistance = -std::numeric_limits<float>::max();
        
        for (const auto& vertex : vertices) {
            // 정점을 월드 좌표계로 변환
            Vector3 worldVertex = rotation.rotate(vertex) + position;
            
            // 방향에 따른 투영 계산
            float distance = direction.dot(worldVertex);
            if (distance > maxDistance) {
                maxDistance = distance;
                furthestPoint = worldVertex;
            }
        }
        
        return furthestPoint;
    }
}

// 내부 접근자
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

// 변환 행렬과 AABB 갱신
void Object3D::update() {
    if (transformDirty) {
        updateTransformMatrix();
    }

    if (aabbDirty) {
        updateWorldAABB();
    }
}