#ifndef OBJECT3D_H
#define OBJECT3D_H

#include <vector>
#include <string>
#include <functional>
#include "Vector3.h"
#include "Matrix3x3.h"
#include "Quaternion.h"
#include "AABB.h"
#include "ConvexDecomposition.h"

class Object3D;
class CollisionManager;

class CollisionInfo {
public:
    Object3D* otherObject;      // 충돌한 다른 객체
    Vector3 contactPoint;       // 충돌 지점
    Vector3 contactNormal;      // 충돌 법선 (이 객체에서 다른 객체 방향)
    float penetrationDepth;     // 침투 깊이

    CollisionInfo();
    CollisionInfo(Object3D* other, const Vector3& point, const Vector3& normal, float depth);
};

class Object3D {
    public:
        // 충돌 이벤트를 위한 콜백 함수 타입 정의
        using CollisionCallback = std::function<void(const CollisionInfo&)>;
    
    private:
        std::string name;           // 객체 이름
    
        // 변환 상태 (위치, 회전, 스케일)
        Vector3 position;           // 위치
        Quaternion rotation;        // 회전 (쿼터니언)
        Vector3 scale;              // 스케일
    
        Matrix3x3 transformMatrix;  // 변환 행렬
        bool transformDirty;        // 변환 행렬 업데이트 필요 여부
    
        AABB localAABB;             // 로컬 좌표계 AABB
        AABB worldAABB;             // 월드 좌표계 AABB
        bool aabbDirty;             // AABB 업데이트 필요 여부
    
        bool isInCollision;         // 충돌 상태 여부
        std::vector<CollisionInfo> collisions;  // 현재 충돌 정보 리스트
    
        // 메시 데이터
        std::vector<Vector3> vertices;  // 정점 배열
        std::vector<Vector3> normals;   // 법선 배열
        std::vector<int> indices;       // 인덱스 배열
    
        // 볼록 분해 결과
        std::vector<ConvexHull> convexHulls;  // 볼록 껍질 배열
        bool isConvexDecomposed;              // 볼록 분해 완료 여부
    
        // 충돌 이벤트 콜백
        CollisionCallback onCollisionEnter;  // 충돌 시작 시 호출
        CollisionCallback onCollisionStay;   // 충돌 지속 시 호출
        CollisionCallback onCollisionExit;   // 충돌 종료 시 호출
    
        friend class CollisionManager;  // CollisionManager가가 접근
    
    public:
        // 생성자 및 소멸자
        Object3D(const std::string& _name = "Object");
        virtual ~Object3D() = default;
    
        // 위치 관련 메서드
        const Vector3& getPosition() const;
        void setPosition(const Vector3& pos);
        void translate(const Vector3& offset);
    
        // 회전 관련 메서드
        const Quaternion& getRotation() const;
        void setRotation(const Quaternion& rot);
        void rotate(const Quaternion& rot);
        void rotateAxis(const Vector3& axis, float angleRadians);
    
        // 스케일 관련 메서드
        const Vector3& getScale() const;
        void setScale(const Vector3& s);
        void setScale(float uniformScale);
    
        // 변환 행렬 연산
        const Matrix3x3& getTransformMatrix();
        void updateTransformMatrix();
    
        // AABB 연산
        void setLocalAABB(const AABB& aabb);
        const AABB& getLocalAABB() const;
        const AABB& getAABB();
        void updateWorldAABB();
    
        // 변환 연산
        Vector3 transformPoint(const Vector3& point);
        Vector3 transformDirection(const Vector3& dir);
        Vector3 inverseTransformPoint(const Vector3& worldPoint);
        Vector3 inverseTransformDirection(const Vector3& worldDir);
    
        // 충돌 관리 (CollisionManager에서 사용)
        bool isColliding() const;
        const std::vector<CollisionInfo>& getCollisions() const;
        void addCollision(const CollisionInfo& collision);
        void removeCollision(Object3D* other);
        void clearCollisions();
    
        // 충돌 이벤트 콜백 설정
        void setOnCollisionEnter(const CollisionCallback& callback);
        void setOnCollisionStay(const CollisionCallback& callback);
        void setOnCollisionExit(const CollisionCallback& callback);
    
        // 메시 데이터 연산
        void setMeshData(const std::vector<Vector3>& verts, 
                        const std::vector<Vector3>& norms,
                        const std::vector<int>& inds);
    
        // 파일 로드 연산
        bool loadFromObjFile(const std::string& filepath);
        
        // 볼록 분해 관련 메서드
        bool computeConvexDecomposition(const std::string& inputObjPath, const std::string& outputObjPath, const VHACDParameters& params);
        bool loadConvexDecomposition(const std::string& filepath);
        void setConvexHulls(const std::vector<ConvexHull>& hulls);
    
        // GJK/EPA 알고리즘용 지원 함수
        Vector3 getSupportPoint(const Vector3& direction) const;
    
        // 접근자
        bool isDecomposed() const;
        const std::vector<ConvexHull>& getConvexHulls() const;
        const std::vector<Vector3>& getVertices() const;
        const std::vector<Vector3>& getNormals() const;
        const std::vector<int>& getIndices() const;
        const std::string& getName() const;
        void setName(const std::string& _name);
    
        // 업데이트 메서드
        void update();
    };
    
    #endif // OBJECT3D_H
