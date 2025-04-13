#include "ConvexDecomposition.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include "../../vhacd/include/VHACD.h"

bool ConvexDecomposition::RunVHACD(const std::string& inputObjPath, 
                                const std::string& outputObjPath, const VHACDParameters& params) 
{
    // 외부 V-HACD 프로세스 실행
    return ExecuteVHACDProcess(inputObjPath, outputObjPath, params);
}

std::vector<ConvexHull> ConvexDecomposition::LoadConvexHulls(const std::string& decomposedObjPath) {
    std::vector<ConvexHull> convexHulls;
    
    std::ifstream file(decomposedObjPath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << decomposedObjPath << std::endl;
        return convexHulls;
    }
    
    std::vector<Vector3> allVertices;
    std::vector<Vector3> tempNormals;  // 법선 정보 (사용되지 않을 수 있음)
    ConvexHull currentHull;
    
    std::string line;
    bool newObjectStarted = false;
    int vertexIndexOffset = 0;
    
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        
        if (prefix == "o" || prefix == "g") {  // 새 객체/그룹
            if (newObjectStarted && !currentHull.vertices.empty()) {
                // 현재 볼록 껍질 저장하고 새 껍질 시작
                convexHulls.push_back(currentHull);
                currentHull = ConvexHull();
                vertexIndexOffset = allVertices.size();
            }
            newObjectStarted = true;
        }
        else if (prefix == "v") {  // 정점
            float x, y, z;
            iss >> x >> y >> z;
            Vector3 vertex(x, y, z);
            allVertices.push_back(vertex);
            currentHull.vertices.push_back(vertex);
        }
        else if (prefix == "f") {  // 면
            if (allVertices.empty()) continue;
            
            std::string v1, v2, v3;
            iss >> v1 >> v2 >> v3;
            
            // 인덱스 파싱 (v/vt/vn 형식 처리)
            auto parseIndex = [](const std::string& str) -> int {
                size_t pos = str.find('/');
                if (pos == std::string::npos) {
                    return std::stoi(str) - 1;  // OBJ는 1부터 인덱싱
                }
                return std::stoi(str.substr(0, pos)) - 1;
            };
            
            int idx1 = parseIndex(v1) - vertexIndexOffset;
            int idx2 = parseIndex(v2) - vertexIndexOffset;
            int idx3 = parseIndex(v3) - vertexIndexOffset;
            
            // 로컬 인덱스 추가
            currentHull.indices.push_back(idx1);
            currentHull.indices.push_back(idx2);
            currentHull.indices.push_back(idx3);
        }
    }
    
    // 마지막 볼록 껍질 추가
    if (!currentHull.vertices.empty()) {
        convexHulls.push_back(currentHull);
    }
    
    return convexHulls;
}

std::vector<ConvexHull> ConvexDecomposition::ComputeConvexDecomposition(
    const std::vector<Vector3>& vertices,
    const std::vector<int>& indices,
    const VHACDParameters& params) {
    
    std::vector<ConvexHull> convexHulls;
    
    if (vertices.empty() || indices.empty()) {
        std::cerr << "Cannot compute convex decomposition: Empty mesh data." << std::endl;
        return convexHulls;
    }
    
    // 메시 데이터를 VHACD 형식으로 변환
    std::vector<float> vhacdPoints;
    std::vector<unsigned int> vhacdTriangles;
    
    // 정점 데이터 변환
    vhacdPoints.reserve(vertices.size() * 3);
    for (const auto& vertex : vertices) {
        vhacdPoints.push_back(vertex.x);
        vhacdPoints.push_back(vertex.y);
        vhacdPoints.push_back(vertex.z);
    }
    
    // 인덱스 데이터 변환
    vhacdTriangles.reserve(indices.size());
    for (const auto& index : indices) {
        vhacdTriangles.push_back(static_cast<unsigned int>(index));
    }
    
    // VHACD 인터페이스 생성
    VHACD::IVHACD* vhacd = VHACD::CreateVHACD();
    
    // 콜백 및 로그 함수 (필요하면 구현)
    class VHACDCallback : public VHACD::IVHACD::IUserCallback {
    public:
        void Update(const double overallProgress, const double stageProgress, 
                   const double operationProgress, const char* const stage,
                   const char* const operation) override {
            // 필요하면 진행 상황 출력 구현
            std::cout << "Progress: " << static_cast<int>(overallProgress * 100.0) << "% - " 
                     << stage << " - " << operation << std::endl;
        }
    };
    
    class VHACDLogger : public VHACD::IVHACD::IUserLogger {
    public:
        void Log(const char* const msg) override {
            // 필요하면 로그 출력 구현
            std::cout << "VHACD: " << msg << std::endl;
        }
    };
    
    VHACDCallback callback;
    VHACDLogger logger;
    
    // VHACD 파라미터 설정
    VHACD::IVHACD::Parameters vhacdParams;
    vhacdParams.m_callback = &callback;
    vhacdParams.m_logger = &logger;

    // 파라미터 매핑 - 정확한 변수명 사용
    vhacdParams.m_maxConvexHulls = params.maxConvexHulls;
    vhacdParams.m_resolution = params.resolution;
    vhacdParams.m_minimumVolumePercentErrorAllowed = params.minimumVolumePercentErrorAllowed;
    vhacdParams.m_maxRecursionDepth = params.maxRecursionDepth;
    vhacdParams.m_planeDownsampling = params.planeDownsampling;
    vhacdParams.m_convexhullDownsampling = params.convexhullDownsampling;
    vhacdParams.m_pca = params.pca;
    vhacdParams.m_mode = params.mode;
    vhacdParams.m_maxNumVerticesPerCH = params.maxNumVerticesPerCH;
    vhacdParams.m_concavity = params.concavity;
    vhacdParams.m_alpha = params.alpha;
    vhacdParams.m_beta = params.beta;
    vhacdParams.m_gamma = params.gamma;
    vhacdParams.m_projectHullVertices = params.projectHullVertices;
    vhacdParams.m_oclAcceleration = params.oclAcceleration;
    
    // V-HACD 분해 실행
    bool success = vhacd->Compute(vhacdPoints.data(), static_cast<uint32_t>(vertices.size()), 
                                 vhacdTriangles.data(), static_cast<uint32_t>(indices.size() / 3), 
                                 vhacdParams);
    
    if (!success) {
        std::cerr << "V-HACD computation failed." << std::endl;
        vhacd->Release();
        return convexHulls;
    }
    
    // 각 볼록 껍질 결과 처리
    uint32_t numHulls = vhacd->GetNConvexHulls();
    convexHulls.resize(numHulls);
    
    for (uint32_t i = 0; i < numHulls; ++i) {
        VHACD::IVHACD::ConvexHull hull;
        vhacd->GetConvexHull(i, hull);
        
        // ConvexHull 객체에 데이터 저장
        ConvexHull& convexHull = convexHulls[i];
        
        // 정점 복사
        convexHull.vertices.resize(hull.m_nPoints);
        for (uint32_t v = 0; v < hull.m_nPoints; ++v) {
            float x = hull.m_points[v * 3];
            float y = hull.m_points[v * 3 + 1];
            float z = hull.m_points[v * 3 + 2];
            convexHull.vertices[v] = Vector3(x, y, z);
        }
        
        // 인덱스 복사
        convexHull.indices.resize(hull.m_nTriangles * 3);
        for (uint32_t t = 0; t < hull.m_nTriangles * 3; ++t) {
            convexHull.indices[t] = static_cast<int>(hull.m_triangles[t]);
        }
    }
    
    // V-HACD 인스턴스 해제
    vhacd->Release();
    
    return convexHulls;
}



bool ConvexDecomposition::ParseObjFile(const std::string& objPath, 
                                      std::vector<Vector3>& outVertices, 
                                      std::vector<std::vector<int>>& outFaces) {
    std::ifstream file(objPath);
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        
        if (prefix == "v") {  // 정점
            float x, y, z;
            iss >> x >> y >> z;
            outVertices.push_back(Vector3(x, y, z));
        }
        else if (prefix == "f") {  // 면
            std::vector<int> face;
            std::string index;
            
            while (iss >> index) {
                // 인덱스 파싱 (v/vt/vn 형식 처리)
                size_t pos = index.find('/');
                int vertexIndex = std::stoi(pos != std::string::npos ? 
                                          index.substr(0, pos) : index) - 1;
                face.push_back(vertexIndex);
            }
            
            if (face.size() >= 3) {
                outFaces.push_back(face);
            }
        }
    }
    
    return !outVertices.empty() && !outFaces.empty();
}

bool ConvexDecomposition::ExecuteVHACDProcess(const std::string& inputPath, 
                                            const std::string& outputPath,
                                            const VHACDParameters& params) {
    // 외부 V-HACD 실행 파일 호출 (TestVHACD 또는 다른 V-HACD 구현체)
    std::string command = "TestVHACD " + inputPath + 
                        " -h " + std::to_string(params.maxConvexHulls) +
                        " -r " + std::to_string(params.resolution) +
                        " -e " + std::to_string(params.minVolumePerCH) +
                        " -d " + std::to_string(params.maxRecursionDepth) +
                        " -v " + std::to_string(params.maxNumVerticesPerCH) +
                        " -o obj" +
                        " > " + outputPath;
    
    // 실제 환경에서는 시스템 명령 대신 라이브러리 API를 직접 호출하는 것이 더 좋습니다
    // 또한 이 구현은 V-HACD 명령줄 도구의 특정 구현에 의존합니다
    int result = system(command.c_str());
    
    return result == 0;
}
}