#ifndef CONVEXDECOMPOSITION_H
#define CONVEXDECOMPOSITION_H

#include <string>
#include <vector>
#include "Vector3.h"
#include "ConvexHull.h"

// V-HACD 파라미터 구조체
struct VHACDParameters {
    unsigned int maxConvexHulls;       // 최대 볼록 껍질 수 (기본값: 32)
    unsigned int resolution;           // 복셀 해상도 (기본값: 100000)
    double minimumVolumePercentErrorAllowed; // 볼륨 백분율 오차 (기본값: 0.01 = 1%)
    unsigned int maxRecursionDepth;    // 최대 재귀 깊이 (기본값: 10)
    bool planeDownsampling;            // 평면 다운샘플링 (기본값: true)
    bool convexhullDownsampling;       // 볼록 껍질 다운샘플링 (기본값: true)
    bool pca;                          // PCA 사용 (기본값: false)
    unsigned int mode;                 // 모드 (0: voxel-based, 1: tetrahedron-based)
    unsigned int maxNumVerticesPerCH;  // 각 볼록 껍질의 최대 정점 수 (기본값: 64)
    double concavity;                  // 오목도 (기본값: 0.001)
    double alpha;                      // 알파 값 (기본값: 0.05)
    double beta;                       // 베타 값 (기본값: 0.05)
    double gamma;                      // 감마 값 (기본값: 0.00125)
    bool projectHullVertices;          // 껍질 정점 투영 (기본값: true)
    bool oclAcceleration;              // OpenCL 가속 사용 (기본값: false)
    
    // 기본 파라미터 설정
    VHACDParameters() 
        : maxConvexHulls(32), 
          resolution(100000),
          minimumVolumePercentErrorAllowed(0.01),
          maxRecursionDepth(10),
          planeDownsampling(true),
          convexhullDownsampling(true),
          pca(false),
          mode(0),  // 0: voxel-based
          maxNumVerticesPerCH(64),
          concavity(0.001),
          alpha(0.05),
          beta(0.05),
          gamma(0.00125),
          projectHullVertices(true),
          oclAcceleration(false) {}
};

class ConvexDecomposition {
public:
    // V-HACD 프로세스를 실행하고 결과를 OBJ 파일로 저장
    static bool RunVHACD(const std::string& inputObjPath, const std::string& outputObjPath, 
        const VHACDParameters& params = VHACDParameters());

    // OBJ 파일에서 볼록 껍질 배열 로드
    static std::vector<ConvexHull> LoadConvexHulls(const std::string& decomposedObjPath);

    // 메시 데이터를 직접 V-HACD로 분해 (API 직접 호출)
    static std::vector<ConvexHull> ComputeConvexDecomposition(
        const std::vector<Vector3>& vertices,
        const std::vector<int>& indices,
        const VHACDParameters& params = VHACDParameters());

private:
    // OBJ 파일에서 정점과 면 정보 추출
    static bool ParseObjFile(const std::string& objPath, 
        std::vector<Vector3>& outVertices, 
        std::vector<std::vector<int>>& outFaces);

    // 외부 V-HACD 실행 파일 호출
    static bool ExecuteVHACDProcess(const std::string& inputPath, 
                const std::string& outputPath,
                const VHACDParameters& params);

};
#endif