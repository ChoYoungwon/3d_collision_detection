# 3D Collision Detection

3D 엔진용 충돌 감지 알고리즘 구현

## 구현 알고리즘

### AABB (Axis Aligned Bounding Box)
- **개념**: 축에 정렬된 사각 박스로 충돌 감지
- **장점**: 빠른 연산 속도
- **단점**: 회전된 객체에서 부정확
- **용도**: 1차 충돌 검사 (Broad Phase)

### SAT (Separating Axis Theorem) 
- **개념**: 분리축 정리를 이용한 OBB 충돌 감지
- **장점**: 회전된 박스 정확한 처리
- **단점**: AABB보다 복잡한 연산
- **용도**: 박스 형태 객체 정밀 검사

### GJK (Gilbert-Johnson-Keerthi)
- **개념**: 볼록한 형태간 최소 거리 계산
- **장점**: 모든 볼록 형태 지원
- **핵심**: Support Function, Minkowski Difference
- **용도**: 복잡한 형태 정밀 검사 (Narrow Phase)

## 🔧 주요 클래스

- **Vector3**: 3D 벡터 연산 (내적, 외적 등)
- **AABB**: 축 정렬 경계 박스
- **OBB**: 회전 가능한 경계 박스  
- **GJK**: 볼록 형태 충돌 감지
- **CollisionManager**: 충돌 관리 시스템

## 성능 최적화

**2단계 충돌 감지**:
1. **Broad Phase**: AABB로 빠른 1차 검사
2. **Narrow Phase**: GJK/SAT로 정밀 검사

## 🛠️ 요구사항
- C++17
- V-HACD (볼록 분해용)
- OpenGL (시뮬레이션 용도)
