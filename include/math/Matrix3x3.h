#include "math/Vector3.h"

namespace Math {

    struct Matrix3x3 {
        float m[3][3];
        
        Matrix3x3();
        Matrix3x3(const Vector3& col1, const Vector3& col2, const Vector3& col3);
        
        // 행렬 연산
        Vector3 operator*(const Vector3& v) const;
        Matrix3x3 operator*(const Matrix3x3& m) const;
        Matrix3x3 transpose() const;
    };
}