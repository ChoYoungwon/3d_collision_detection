#ifndef SAT_H
#define SAT_H

#include "../math/Vector3.h"
#include "../geometry/OBB.h"
#include <vector>

namespace Collision {

    class SAT {
    public:
        // �� OBB �� �浹 ���θ� �Ǵ��ϴ� ���� �Լ�
        static bool TestOBBCollision(const OBB& obbA, const OBB& obbB);

    private:
        // �־��� ��(axis) �󿡼� �� OBB�� ����(projection)�� ��ġ���� �˻��ϴ� �Լ�
        static bool OverlapOnAxis(const OBB& obbA, const OBB& obbB, const Vector3& axis);
    };

} // namespace Collision

#endif // SAT_H