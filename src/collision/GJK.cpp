#include "GJK.h"
#include <cmath>
#include <algorithm>

namespace Collision {

    // ���� �Լ�: �� ��ü���� ���� �־��� ����� �ݴ� ������ �ִ� �Ÿ��� ���� ���� ���� ��, �� ���� ��(= Minkowski ��)�� ��ȯ
    Vector3 GJK::Support(const ConvexHull& shapeA, const ConvexHull& shapeB, const Vector3& dir) {
        // shapeA���� ���� dir�� ���� �� ��
        Vector3 pointA = shapeA.GetFurthestPoint(dir);
        // shapeB������ -dir �������� ���� �� �� (��, �ݴ���)
        Vector3 pointB = shapeB.GetFurthestPoint(-dir);
        return pointA - pointB;
    }

    bool GJK::Intersect(const ConvexHull& shapeA, const ConvexHull& shapeB) {
        // 1. �ʱ� ���� ���� (������ ����)
        Vector3 direction(1, 0, 0);
        // 2. ������ ���� ���� �ܼ�ü�� �߰�
        std::vector<Vector3> simplex;
        simplex.push_back(Support(shapeA, shapeB, direction));
        // 3. ������ ���ϴ� �˻� ����: �ܼ�ü�� ù ���� �ݴ� ����
        direction = -simplex[0];

        // 4. �ݺ� ����: ���ο� ���� ���� ����� �ܼ�ü�� �߰��ϸ鼭 ���� ���� ���θ� �Ǵ�
        while (true) {
            Vector3 newPoint = Support(shapeA, shapeB, direction);
            // ���� �� ���� ���� ���� ���⿡�� ������ �Ѿ�� ���ϸ� �浹���� �ʴ� ������ �Ǵ�
            if (newPoint.Dot(direction) <= 0) {
                return false;
            }
            // �ܼ�ü�� �� �� �߰�
            simplex.push_back(newPoint);
            // �ܼ�ü�� �м��Ͽ� (������Ʈ) ������ ���ԵǾ����� Ȯ��
            if (HandleSimplex(simplex, direction)) {
                return true;
            }
        }
    }

    // HandleSimplex �Լ�: �ܼ�ü�� �� ������ ���� �� ���� ���(line, triangle, tetrahedron)�� ó��
    bool GJK::HandleSimplex(std::vector<Vector3>& simplex, Vector3& direction) {
        if (simplex.size() == 2) {
            // ---------- ����(simplex = {A, B}) ó�� ----------
            // �ܼ�ü�� �������� �߰��� ���� A, �� ���� ���� B�� �����մϴ�.
            Vector3 A = simplex.back();
            Vector3 B = simplex.front();
            Vector3 AB = B - A;
            Vector3 AO = -A; // �������� ���ϴ� ����

            // A�� AB ���п� ������(������ ���ϴ�) ���͸� ã��
            direction = AB.Cross(AO).Cross(AB);
            // ���� direction�� 0 ���Ͷ��, AB�� ������ ������ ���͸� ����
            if (direction.LengthSquared() < 1e-6f) {
                direction = Vector3(-AB.y, AB.x, 0);
            }
        }
        else if (simplex.size() == 3) {
            // ---------- �ﰢ��(simplex = {A, B, C}) ó�� ----------
            // A: ���� �ֱ� �߰��� ��
            Vector3 A = simplex[2];
            Vector3 B = simplex[1];
            Vector3 C = simplex[0];
            Vector3 AB = B - A;
            Vector3 AC = C - A;
            Vector3 AO = -A;

            // �ﰢ���� ����� ���� (����ȭ ��)
            Vector3 ABC = AB.Cross(AC);

            // Edge AB�� ���� ���� ���� (�ﰢ���� ��� ������, ���� ������ ����)
            Vector3 ABPerp = ABC.Cross(AB);
            if (ABPerp.Dot(AO) > 0) {
                // ������ edge AB �ʿ� �ִٸ�, C�� �ܼ�ü���� �����ϰ�
                simplex.erase(simplex.begin()); // C ���� (���� �� ��)
                direction = AB.Cross(AO).Cross(AB);
                return false;
            }

            // Edge AC�� ���� ���� ����
            Vector3 ACPerp = AC.Cross(ABC);
            if (ACPerp.Dot(AO) > 0) {
                // ������ edge AC �ʿ� �ִٸ�, B�� �ܼ�ü���� �����մϴ�.
                simplex.erase(simplex.begin() + 1); // B ����
                direction = AC.Cross(AO).Cross(AC);
                return false;
            }

            // ������ �ﰢ�� ���ο� �ְų� �ﰢ�� ��� �� ������, �˻� ������ �ﰢ���� ���� ������ �����մϴ�.
            if (ABC.Dot(AO) > 0) {
                direction = ABC;
            }
            else {
                // ���� ������ ���� �ùٸ� ������ ����
                direction = -ABC;
                // �ܼ�ü�� ������ �ڹٲ㼭 ������ ����� �򵵷� ��
                std::swap(simplex[0], simplex[1]);
            }
        }
        else if (simplex.size() == 4) {
            // ---------- ���ü(simplex = {A, B, C, D}) ó�� ----------
            // A: ���� �ֱ� �߰��� ��
            Vector3 A = simplex[3];
            Vector3 B = simplex[2];
            Vector3 C = simplex[1];
            Vector3 D = simplex[0];
            Vector3 AO = -A;

            // �� ���� ���� ��� (A�� �������� �� ��)
            Vector3 ABC = (B - A).Cross(C - A);
            Vector3 ACD = (C - A).Cross(D - A);
            Vector3 ADB = (D - A).Cross(B - A);

            // ������ �ش� ���� ������ ������, �ܼ�ü�� �ش� ���� ����� ���̰�
            // �˻� ������ �� ���� �������� �����մϴ�.
            if (ABC.Dot(AO) > 0) {
                // D�� �Ⱒ�մϴ�.
                simplex.erase(simplex.begin()); // D ����
                direction = ABC;
                return false;
            }
            if (ACD.Dot(AO) > 0) {
                // B�� �Ⱒ�մϴ�.
                simplex.erase(simplex.begin() + 2); // B ����
                direction = ACD;
                return false;
            }
            if (ADB.Dot(AO) > 0) {
                // C�� �Ⱒ�մϴ�.
                simplex.erase(simplex.begin() + 1); // C ����
                direction = ADB;
                return false;
            }
            // ������ ��� ���� ���ο� �ִٸ�, �� ��ü�� �浹 ���Դϴ�.
            return true;
        }
        return false;
    }

} // namespace Collision