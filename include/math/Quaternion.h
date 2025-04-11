#include "math/Vector3.h"
#include "math/Matrix3x3.h"

namespace Math {

    struct Quaternion {
        float w, x, y, z;
        
        Quaternion() : w(1), x(0), y(0), z(0) {}
        Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
        
        // 쿼터니언 생성 및 변환
        static Quaternion fromAxisAngle(const Vector3& axis, float angle);
        Matrix3x3 toRotationMatrix() const;
        
        // 쿼터니언 연산
        Quaternion operator*(const Quaternion& q) const;
        Vector3 rotate(const Vector3& v) const;
        Quaternion normalized() const;
    };
}