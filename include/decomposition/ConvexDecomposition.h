#ifndef CONVEXDECOMPOSITION_H
#define CONVEXDECOMPOSITION_H

#include <string>
#include <vector>
#include "Vector3.h"
#include "ConvexHull.h"

// V-HACD 파라미터 구조체
struct VHACDParameters {
    unsigned int maxConvexHulls;                 // 최대 볼록 껍질 수 (기본값: 64)
    unsigned int resolution;                     // 복셀 해상도 (기본값: 400000)
    double minimumVolumePercentErrorAllowed;     // 볼륨 백분율 오차 (기본값: 1.0 = 1%)
    unsigned int maxRecursionDepth;              // 최대 재귀 깊이 (기본값: 10)
    bool shrinkWrap;                             // 소스 메시에 복셀 위치를 맞출지 여부 (기본값: true)
    unsigned int fillMode;                       // 채우기 모드 (0: FLOOD_FILL, 1: SURFACE_ONLY, 2: RAYCAST)
    unsigned int maxNumVerticesPerCH;            // 각 볼록 껍질의 최대 정점 수 (기본값: 64)
    bool asyncACD;                               // 비동기 실행 여부 (기본값: true)
    unsigned int minEdgeLength;                  // 최소 복셀 에지 길이 (기본값: 2)
    bool findBestPlane;
    double minVolumePerCH;                       // 최소 볼륨 비율 (ExecuteVHACDProcess에서 사용)
    
    // 기본 파라미터 설정
    VHACDParameters() 
        : maxConvexHulls(64), 
          resolution(400000),
          minimumVolumePercentErrorAllowed(1.0),
          maxRecursionDepth(10),
          shrinkWrap(true),
          fillMode(0),  // FLOOD_FILL
          maxNumVerticesPerCH(64),
          asyncACD(true),
          minEdgeLength(2),
          findBestPlane(false),
          minVolumePerCH(0.01) {}
};

class ConvexDecomposition {
public:
    // V-HACD 프로세스를 실행하고 결과를 OBJ 파일로 저장
    static bool RunVHACD(
        const std::string& inputObjPath, 
        const std::string& outputObjPath, 
        const VHACDParameters& params = VHACDParameters()
    );

    // 분해된 OBJ 파일에서 볼록 껍질 객체들을 로드
    static std::vector<ConvexHull> LoadConvexHulls(
        const std::string& decomposedObjPath
    );

    // 메시 데이터를 직접 V-HACD로 분해 (API 직접 호출)
    // static std::vector<ConvexHull> ComputeConvexDecomposition(
    //     const std::vector<Vector3>& vertices,
    //     const std::vector<int>& indices,
    //     const VHACDParameters& params = VHACDParameters()
    // );

    // OBJ 파일에서 정점과 면 정보 추출
    static bool ParseObjFile(
        const std::string& objPath,
        // 정점 데이터를 저장할 벡터(출력)
        std::vector<Vector3>& outVertices,
        // 면 데이터를 저장할 벡터
        std::vector<std::vector<int>>& outFaces
    );

private:
    // 외부 V-HACD 실행 파일 호출
    static bool ExecuteVHACDProcess(
        const std::string& inputPath, 
        const std::string& outputPath,
        const VHACDParameters& params
    );
};

#endif