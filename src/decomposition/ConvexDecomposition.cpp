#include "ConvexDecomposition.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include "../../vhacd/include/VHACD.h"

// V-HACD 프로세스를 실행하고 결과를 OBJ 파일로 저장
bool ConvexDecomposition::RunVHACD(
    const std::string& inputObjPath, 
    const std::string& outputObjPath,
    const VHACDParameters& params
) { // 외부 V-HACD 프로세스 실행
    return ExecuteVHACDProcess(inputObjPath, outputObjPath, params);
}

// 분해된 OBJ 파일에서 볼록 껍질 객체들을 로드
std::vector<ConvexHull> ConvexDecomposition::LoadConvexHulls(
    const std::string& decomposedObjPath
) {
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

// OBJ 파일에서 정점과 면 정보 추출
bool ConvexDecomposition::ParseObjFile(
    const std::string& objPath, 
    std::vector<Vector3>& outVertices, 
    std::vector<std::vector<int>>& outFaces
) {
    std::ifstream file(objPath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << objPath << std::endl;
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

// 외부 V-HACD 실행 파일 호출
bool ConvexDecomposition::ExecuteVHACDProcess(
    const std::string& inputPath, 
    const std::string& outputPath,
    const VHACDParameters& params
) {
    // 외부 V-HACD 실행 파일 호출 (TestVHACD 또는 다른 V-HACD 구현체)
    std::string command = "./TestVHACD " + inputPath + 
                        " -h " + std::to_string(params.maxConvexHulls) +
                        " -r " + std::to_string(params.resolution) +
                        " -e " + std::to_string(params.minVolumePerCH) +
                        " -d " + std::to_string(params.maxRecursionDepth) +
                        " -v " + std::to_string(params.maxNumVerticesPerCH);
    
    // 실제 환경에서는 시스템 명령 대신 라이브러리 API를 직접 호출하는 것이 더 좋습니다
    // 또한 이 구현은 V-HACD 명령줄 도구의 특정 구현에 의존합니다
    int result = system(command.c_str());

    // 성공하면 파일 이름 변경
    if (result == 0) {
        // C++17 파일 시스템 사용
        #if __cplusplus >= 201703L
            std::filesystem::rename("decomp.obj", outputPath);
            
            // MTL 파일도 이름 변경 (있는 경우)
            std::string mtlOutput = outputPath.substr(0, outputPath.find_last_of(".")) + ".mtl";
            if (std::filesystem::exists("decomp.mtl")) {
                std::filesystem::rename("decomp.mtl", mtlOutput);
            }
        // C++17 이전 버전
        #else
            // 파일 이름 변경
            std::rename("decomp.obj", outputPath.c_str());
            
            // MTL 파일도 이름 변경 (있는 경우)
            std::string mtlOutput = outputPath.substr(0, outputPath.find_last_of(".")) + ".mtl";
            std::ifstream testMtl("decomp.mtl");
            if (testMtl.good()) {
                testMtl.close();
                std::rename("decomp.mtl", mtlOutput.c_str());
            }
        #endif
        
        return true;
    }
    
    return false;
}