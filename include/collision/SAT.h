#ifndef SAT_H
#define SAT_H

#include "../math/Vector3.h"
#include "../geometry/OBB.h"
#include <vector>

namespace Collision {

    class SAT {
    public:
        // 두 OBB 간 충돌 여부를 판단하는 정적 함수
        static bool TestOBBCollision(const OBB& obbA, const OBB& obbB);

    private:
        // 주어진 축(axis) 상에서 두 OBB의 투영(projection)이 겹치는지 검사하는 함수
        static bool OverlapOnAxis(const OBB& obbA, const OBB& obbB, const Vector3& axis);
    };

} // namespace Collision

#endif // SAT_H