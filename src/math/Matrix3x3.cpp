#include "math/Matrix3x3.h"

namespace Math {

    Matrix3x3::Matrix3x3() {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                m[i][j] = (i == j) ? 1.0f : 0.0f;  // 단위 행렬
    }

    Matrix3x3::Matrix3x3(const Vector3& col1, const Vector3& col2, const Vector3& col3) {
        m[0][0] = col1.x; m[0][1] = col2.x; m[0][2] = col3.x;
        m[1][0] = col1.y; m[1][1] = col2.y; m[1][2] = col3.y;
        m[2][0] = col1.z; m[2][1] = col2.z; m[2][2] = col3.z;
    }

    Vector3 Matrix3x3::operator*(const Vector3& v) const {
        return Vector3(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        );
    }

    Matrix3x3 Matrix3x3::operator*(const Matrix3x3& other) const {
        Matrix3x3 result;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                result.m[row][col] =
                    m[row][0] * other.m[0][col] +
                    m[row][1] * other.m[1][col] +
                    m[row][2] * other.m[2][col];
            }
        }
        return result;
    }

    Matrix3x3 Matrix3x3::transpose() const {
        Matrix3x3 result;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                result.m[i][j] = m[j][i];
        return result;
    }
}