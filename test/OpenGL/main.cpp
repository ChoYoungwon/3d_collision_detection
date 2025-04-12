#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cfloat>

// 창 크기
const unsigned int SCR_WIDTH = 1200;
const unsigned int SCR_HEIGHT = 800;

// 카메라 설정
glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 5.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
glm::vec3 centerPoint = glm::vec3(0.0f);
float cameraDistance = 5.0f;

// 마우스/키보드 제어를 위한 변수
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
float yaw = -90.0f;
float pitch = 0.0f;
bool firstMouse = true;
bool wireframe = false;
bool mousePressed = false;

// 시간 관리
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// OBJ 파일에서 삼각형 메시 로드
struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    GLuint VAO, VBO, EBO;
    glm::vec4 color;
    float opacity;
    
    Mesh(const glm::vec4& color = glm::vec4(1.0f), float opacity = 1.0f) 
        : color(color), opacity(opacity) {}
    
    void setupMesh() {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
  
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);  

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

        // 정점 위치
        glEnableVertexAttribArray(0);   
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
        
        // 정점 법선
        glEnableVertexAttribArray(1);   
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

        glBindVertexArray(0);
    }
    
    void draw(GLuint shaderProgram) const {
        glUniform4fv(glGetUniformLocation(shaderProgram, "meshColor"), 1, glm::value_ptr(color));
        glUniform1f(glGetUniformLocation(shaderProgram, "opacity"), opacity);
        
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
};

// 여러 파트를 포함하는 OBJ 로드 함수
std::vector<Mesh> loadOBJPart(const char* path) {
    std::vector<Mesh> meshes;
    
    std::vector<glm::vec3> temp_vertices;
    std::vector<glm::vec3> temp_normals;
    
    std::ifstream file(path);
    if (!file) {
        std::cerr << "파일을 열 수 없습니다: " << path << std::endl;
        return meshes;
    }
    
    Mesh currentMesh(glm::vec4(1.0f, 0.0f, 0.0f, 0.5f));  // 기본 빨간색 반투명
    std::string line;
    bool newObjectStarted = false;
    
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        
        if (prefix == "o" || prefix == "g") {  // 새 객체/그룹 시작
            if (newObjectStarted && !currentMesh.vertices.empty()) {
                // 현재 메시를 목록에 추가하고 새 메시 시작
                meshes.push_back(currentMesh);
                currentMesh = Mesh(glm::vec4(
                    rand() / (float)RAND_MAX,  // 랜덤 색상 부여
                    rand() / (float)RAND_MAX,
                    rand() / (float)RAND_MAX,
                    0.5f
                ));
            }
            newObjectStarted = true;
            std::string name;
            iss >> name;
            std::cout << "새 파트 발견: " << name << std::endl;
        }
        else if (prefix == "v") {  // 정점
            glm::vec3 vertex;
            iss >> vertex.x >> vertex.y >> vertex.z;
            temp_vertices.push_back(vertex);
        }
        else if (prefix == "vn") {  // 법선
            glm::vec3 normal;
            iss >> normal.x >> normal.y >> normal.z;
            temp_normals.push_back(normal);
        }
        else if (prefix == "f") {  // 면
            std::string vertex1, vertex2, vertex3;
            iss >> vertex1 >> vertex2 >> vertex3;
            
            std::vector<unsigned int> vertexIndices, normalIndices;
            
            auto parseVertex = [&](std::string vertexStr) {  // 참조 제거, 값으로 전달
                std::replace(vertexStr.begin(), vertexStr.end(), '/', ' ');
                std::istringstream iss(vertexStr);
                unsigned int vertexIndex = 0, normalIndex = 0;
                unsigned int temp;
                
                iss >> vertexIndex;
                if (iss.peek() == ' ') iss >> temp;
                iss >> normalIndex;
                
                return std::make_pair(vertexIndex, normalIndex);
            };
            
            // C++11 스타일로 구조 변경
            auto v1Result = parseVertex(vertex1);
            auto v2Result = parseVertex(vertex2);
            auto v3Result = parseVertex(vertex3);
            
            unsigned int v1Index = v1Result.first, n1Index = v1Result.second;
            unsigned int v2Index = v2Result.first, n2Index = v2Result.second;
            unsigned int v3Index = v3Result.first, n3Index = v3Result.second;
            
            // 나머지 코드는 이전과 동일
            Vertex v1, v2, v3;
            v1.position = temp_vertices[v1Index-1];
            v2.position = temp_vertices[v2Index-1];
            v3.position = temp_vertices[v3Index-1];
            
            v1.normal = (n1Index-1 < temp_normals.size()) ? temp_normals[n1Index-1] : glm::vec3(0.0f, 0.0f, 1.0f);
            v2.normal = (n2Index-1 < temp_normals.size()) ? temp_normals[n2Index-1] : glm::vec3(0.0f, 0.0f, 1.0f);
            v3.normal = (n3Index-1 < temp_normals.size()) ? temp_normals[n3Index-1] : glm::vec3(0.0f, 0.0f, 1.0f);
            
            unsigned int index1 = currentMesh.vertices.size();
            currentMesh.vertices.push_back(v1);
            currentMesh.vertices.push_back(v2);
            currentMesh.vertices.push_back(v3);
            
            currentMesh.indices.push_back(index1);
            currentMesh.indices.push_back(index1 + 1);
            currentMesh.indices.push_back(index1 + 2);
        }
    }
    
    // 마지막 메시 추가
    if (!currentMesh.vertices.empty()) {
        meshes.push_back(currentMesh);
    }
    
    std::cout << "총 " << meshes.size() << "개의 볼록 파트를 로드했습니다." << std::endl;
    
    return meshes;
}

// OBJ 파일 로드 함수
bool loadOBJ(const char* path, Mesh& mesh) {
    std::vector<glm::vec3> temp_vertices;
    std::vector<glm::vec3> temp_normals;
    std::vector<unsigned int> vertexIndices, normalIndices;
    
    std::ifstream file(path);
    if (!file) {
        std::cerr << "파일을 열 수 없습니다: " << path << std::endl;
        return false;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        
        if (prefix == "v") { // 정점
            glm::vec3 vertex;
            iss >> vertex.x >> vertex.y >> vertex.z;
            temp_vertices.push_back(vertex);
        } 
        else if (prefix == "vn") { // 법선
            glm::vec3 normal;
            iss >> normal.x >> normal.y >> normal.z;
            temp_normals.push_back(normal);
        } 
        else if (prefix == "f") { // 면
            std::string vertex1, vertex2, vertex3;
            iss >> vertex1 >> vertex2 >> vertex3;
            
            // 정점/텍스처/법선 인덱스 파싱
            std::replace(vertex1.begin(), vertex1.end(), '/', ' ');
            std::replace(vertex2.begin(), vertex2.end(), '/', ' ');
            std::replace(vertex3.begin(), vertex3.end(), '/', ' ');
            
            std::istringstream v1(vertex1), v2(vertex2), v3(vertex3);
            unsigned int vertexIndex, normalIndex;
            unsigned int temp;
            
            v1 >> vertexIndex;
            if (v1.peek() == ' ') {
                v1 >> temp; // 텍스처 건너뛰기
            }
            v1 >> normalIndex;
            vertexIndices.push_back(vertexIndex - 1);
            normalIndices.push_back(normalIndex - 1);
            
            v2 >> vertexIndex;
            if (v2.peek() == ' ') {
                v2 >> temp; // 텍스처 건너뛰기
            }
            v2 >> normalIndex;
            vertexIndices.push_back(vertexIndex - 1);
            normalIndices.push_back(normalIndex - 1);
            
            v3 >> vertexIndex;
            if (v3.peek() == ' ') {
                v3 >> temp; // 텍스처 건너뛰기
            }
            v3 >> normalIndex;
            vertexIndices.push_back(vertexIndex - 1);
            normalIndices.push_back(normalIndex - 1);
        }
    }
    
    // 인덱스로부터 실제 정점 데이터 생성
    for (unsigned int i = 0; i < vertexIndices.size(); i++) {
        Vertex vertex;
        
        // 정점 위치
        if (vertexIndices[i] < temp_vertices.size()) {
            vertex.position = temp_vertices[vertexIndices[i]];
        }
        
        // 법선 벡터
        if (i < normalIndices.size() && normalIndices[i] < temp_normals.size()) {
            vertex.normal = temp_normals[normalIndices[i]];
        } else {
            // 법선 정보가 없으면 임시 법선 생성
            vertex.normal = glm::vec3(0.0f, 0.0f, 1.0f);
        }
        
        mesh.vertices.push_back(vertex);
        mesh.indices.push_back(i);
    }
    
    std::cout << "모델 로드 완료 '" << path << "': " << mesh.vertices.size() << " 정점, " 
              << mesh.indices.size() / 3 << " 면" << std::endl;
    
    return true;
}

bool loadSTLASCII(const char* path, Mesh& mesh) {
    std::ifstream file(path);
    if (!file) {
        std::cerr << "파일을 열 수 없습니다: " << path << std::endl;
        return false;
    }
    
    std::string line, token;
    std::string solidName;
    
    // 첫 줄 읽기 (solid 확인)
    std::getline(file, line);
    std::istringstream iss(line);
    iss >> token;
    
    if (token != "solid") {
        std::cerr << "STL 파일 형식이 올바르지 않습니다: " << path << " (solid로 시작하지 않음)" << std::endl;
        return false;
    }
    
    // solid 이름 읽기
    iss >> solidName;
    std::cout << "STL ASCII 파일 로드 중: " << path << " (솔리드 이름: " << solidName << ")" << std::endl;
    
    glm::vec3 normal;
    glm::vec3 vertex;
    uint32_t triangleCount = 0;
    
    // 파일 파싱
    while (std::getline(file, line)) {
        std::istringstream lineStream(line);
        lineStream >> token;
        
        if (token == "facet") {
            // 법선 벡터 읽기
            lineStream >> token; // "normal"
            lineStream >> normal.x >> normal.y >> normal.z;
        }
        else if (token == "vertex") {
            // 정점 읽기
            lineStream >> vertex.x >> vertex.y >> vertex.z;
            
            Vertex meshVertex;
            meshVertex.position = vertex;
            meshVertex.normal = normal;
            
            mesh.vertices.push_back(meshVertex);
            mesh.indices.push_back(mesh.vertices.size() - 1);
        }
        else if (token == "endfacet") {
            // 삼각형 완료
            triangleCount++;
        }
        else if (token == "endsolid") {
            // 파일 끝
            break;
        }
    }
    
    if (mesh.vertices.empty()) {
        std::cerr << "STL 파일에서 유효한 삼각형을 찾을 수 없습니다: " << path << std::endl;
        return false;
    }
    
    std::cout << "STL ASCII 모델 로드 완료 '" << path << "': " << mesh.vertices.size() << " 정점, " 
              << triangleCount << " 면" << std::endl;
    
    return true;
}

bool loadSTLBinary(const char* path, Mesh& mesh) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        std::cerr << "파일을 열 수 없습니다: " << path << std::endl;
        return false;
    }
    
    // 파일 크기 확인
    file.seekg(0, std::ios::end);
    std::streampos fileSize = file.tellg();
    file.seekg(0, std::ios::beg);
    
    if (fileSize < 84) {  // 최소 STL 파일 크기 (헤더 80바이트 + 삼각형 개수 4바이트)
        std::cerr << "STL 파일 형식이 올바르지 않습니다: " << path << " (파일 크기가 너무 작습니다)" << std::endl;
        return false;
    }
    
    // STL 헤더 읽기 (디버깅용)
    char header[81] = {0};  // 80바이트 + NULL 종료 문자
    file.read(header, 80);
    std::cout << "STL 헤더: " << header << std::endl;
    
    // 삼각형 개수 읽기
    uint32_t triangleCount;
    file.read(reinterpret_cast<char*>(&triangleCount), sizeof(uint32_t));
    
    // 삼각형 개수 검증
    std::streamsize expectedSize = 84 + (triangleCount * 50);  // 헤더(80) + 개수(4) + 삼각형(50 * 개수)
    if (expectedSize > fileSize) {
        std::cerr << "STL 파일 형식이 올바르지 않습니다: " << path 
                  << " (예상 크기: " << expectedSize << ", 실제 크기: " << fileSize << ")" << std::endl;
        return false;
    }
    
    std::cout << "STL 파일 로드 중: " << path << " (삼각형 개수: " << triangleCount << ")" << std::endl;
    
    // 각 삼각형 데이터 읽기
    for (uint32_t i = 0; i < triangleCount; i++) {
        // 법선 벡터 읽기
        float nx, ny, nz;
        file.read(reinterpret_cast<char*>(&nx), sizeof(float));
        file.read(reinterpret_cast<char*>(&ny), sizeof(float));
        file.read(reinterpret_cast<char*>(&nz), sizeof(float));
        glm::vec3 normal(nx, ny, nz);
        
        // 세 정점 읽기
        for (int j = 0; j < 3; j++) {
            float x, y, z;
            file.read(reinterpret_cast<char*>(&x), sizeof(float));
            file.read(reinterpret_cast<char*>(&y), sizeof(float));
            file.read(reinterpret_cast<char*>(&z), sizeof(float));
            
            Vertex vertex;
            vertex.position = glm::vec3(x, y, z);
            vertex.normal = normal;
            
            mesh.vertices.push_back(vertex);
            mesh.indices.push_back(mesh.vertices.size() - 1);
        }
        
        // 속성 바이트 카운트 건너뛰기 (2 바이트)
        uint16_t attrByteCount;
        file.read(reinterpret_cast<char*>(&attrByteCount), sizeof(uint16_t));
    }
    
    if (mesh.vertices.empty()) {
        std::cerr << "STL 파일에서 유효한 삼각형을 찾을 수 없습니다: " << path << std::endl;
        return false;
    }
    
    std::cout << "STL 모델 로드 완료 '" << path << "': " << mesh.vertices.size() << " 정점, " 
              << triangleCount << " 면" << std::endl;
    
    return true;
}

bool loadSTL(const char* path, Mesh& mesh) {
    // 먼저 텍스트 모드로 열어서 ASCII STL인지 확인
    std::ifstream testFile(path);
    if (!testFile) {
        std::cerr << "파일을 열 수 없습니다: " << path << std::endl;
        return false;
    }
    
    std::string firstWord;
    testFile >> firstWord;
    testFile.close();
    
    if (firstWord == "solid") {
        // ASCII STL 파일
        return loadSTLASCII(path, mesh);
    } else {
        // 바이너리 STL 파일로 처리 시도
        return loadSTLBinary(path, mesh);
    }
}

// 셰이더 컴파일 함수
GLuint compileShader(const char* vertexShaderSource, const char* fragmentShaderSource) {
    // 정점 셰이더 컴파일
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);
    
    // 컴파일 오류 확인
    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cerr << "정점 셰이더 컴파일 오류: " << infoLog << std::endl;
    }
    
    // 프래그먼트 셰이더 컴파일
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);
    
    // 컴파일 오류 확인
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cerr << "프래그먼트 셰이더 컴파일 오류: " << infoLog << std::endl;
    }
    
    // 셰이더 프로그램 생성
    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    
    // 링크 오류 확인
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cerr << "셰이더 프로그램 링크 오류: " << infoLog << std::endl;
    }
    
    // 셰이더 삭제
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    
    return shaderProgram;
}

// 마우스 버튼 콜백 함수 추가
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            mousePressed = true;
            firstMouse = true;
        }
        else if (action == GLFW_RELEASE)
            mousePressed = false;
    }
}

// 마우스 입력 콜백
void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    if (!mousePressed)
        return;

    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }
    
    float xoffset = -(xpos - lastX);
    float yoffset = -(lastY - ypos);
    lastX = xpos;
    lastY = ypos;
    
    float sensitivity = 0.1f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;
    
    yaw += xoffset;
    pitch += yoffset;
    
    // 피치 제한
    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;
    
    // glm::vec3 front;
    // front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    // front.y = sin(glm::radians(pitch));
    // front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    // cameraFront = glm::normalize(front);
}

// 스크롤 입력 콜백
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    // 줌 인/아웃
    cameraDistance -= yoffset * 0.5f;

    // 최소/최대 거리 제한
    if (cameraDistance < 0.5f)
        cameraDistance = 0.5f;
    if (cameraDistance > 20.0f)
        cameraDistance = 20.0f;
}

// 카메라 위치 업데이트 함수
void updateCameraPosition() {
    // 구면 좌표계 사용
    float camX = sin(glm::radians(yaw)) * cos(glm::radians(pitch)) * cameraDistance;
    float camY = sin(glm::radians(pitch)) * cameraDistance;
    float camZ = cos(glm::radians(yaw)) * cos(glm::radians(pitch)) * cameraDistance;
    
    cameraPos = centerPoint + glm::vec3(camX, camY, camZ);
    cameraFront = glm::normalize(centerPoint - cameraPos);
}

// 윈도우 종료 콜백
void window_close_callback(GLFWwindow* window)
{
    glfwSetWindowShouldClose(window, GLFW_TRUE);
}

// 키 입력 처리
void processInput(GLFWwindow *window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    
    float cameraSpeed = 2.5f * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos += cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    
    // 와이어프레임 모드 전환 (스페이스바)
    static bool spacePressed = false;
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        if (!spacePressed) {
            wireframe = !wireframe;
            spacePressed = true;
        }
    } else {
        spacePressed = false;
    }
}

// 모델 중앙 정렬 및 크기 조정 함수 추가
void centerAndScaleModel(Mesh& mesh) {
    if (mesh.vertices.empty()) {
        return;
    }
    
    // 바운딩 박스 계산
    glm::vec3 min(FLT_MAX), max(-FLT_MAX);
    for (const auto& vertex : mesh.vertices) {
        min.x = std::min(min.x, vertex.position.x);
        min.y = std::min(min.y, vertex.position.y);
        min.z = std::min(min.z, vertex.position.z);
        
        max.x = std::max(max.x, vertex.position.x);
        max.y = std::max(max.y, vertex.position.y);
        max.z = std::max(max.z, vertex.position.z);
    }
    
    // 모델 중심 계산
    glm::vec3 center = (min + max) * 0.5f;
    
    // 모델 크기 계산 (가장 긴 차원 기준)
    float maxSize = std::max(std::max(max.x - min.x, max.y - min.y), max.z - min.z);
    float scale = 2.0f / maxSize; // 모델을 [-1, 1] 범위에 맞추기
    
    std::cout << "모델 중앙 정렬 및 크기 조정 - 중심: (" << center.x << ", " << center.y 
              << ", " << center.z << "), 크기: " << maxSize << ", 스케일: " << scale << std::endl;
    
    // 모든 정점 조정
    for (auto& vertex : mesh.vertices) {
        vertex.position = (vertex.position - center) * scale;
    }
}

// 모델 로드 후 정보 출력
void printModelInfo(const Mesh& mesh) {
    // 모델의 바운딩 박스 계산
    glm::vec3 min(FLT_MAX), max(-FLT_MAX);
    
    for (const auto& vertex : mesh.vertices) {
        min.x = std::min(min.x, vertex.position.x);
        min.y = std::min(min.y, vertex.position.y);
        min.z = std::min(min.z, vertex.position.z);
        
        max.x = std::max(max.x, vertex.position.x);
        max.y = std::max(max.y, vertex.position.y);
        max.z = std::max(max.z, vertex.position.z);
    }
    
    // 모델 중심과 크기 계산
    glm::vec3 center = (min + max) * 0.5f;
    glm::vec3 size = max - min;
    
    std::cout << "모델 정보:" << std::endl;
    std::cout << "  바운딩 박스 최소: (" << min.x << ", " << min.y << ", " << min.z << ")" << std::endl;
    std::cout << "  바운딩 박스 최대: (" << max.x << ", " << max.y << ", " << max.z << ")" << std::endl;
    std::cout << "  중심점: (" << center.x << ", " << center.y << ", " << center.z << ")" << std::endl;
    std::cout << "  크기: (" << size.x << ", " << size.y << ", " << size.z << ")" << std::endl;
}

int main(int argc, char* argv[]) {
    // GLFW 초기화
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    // 창 생성
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "3D 객체 및 볼록화 시각화", NULL, NULL);
    if (window == NULL) {
        std::cerr << "GLFW 창 생성 실패" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetWindowCloseCallback(window, window_close_callback);
    
    // 마우스 캡처
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    
    // GLEW 초기화
    if (glewInit() != GLEW_OK) {
        std::cerr << "GLEW 초기화 실패" << std::endl;
        return -1;
    }
    
    // 깊이 테스트 및 블렌딩 활성화
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // 정점 셰이더 소스
    const char* vertexShaderSource = "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "layout (location = 1) in vec3 aNormal;\n"
        "out vec3 Normal;\n"
        "out vec3 FragPos;\n"
        "uniform mat4 model;\n"
        "uniform mat4 view;\n"
        "uniform mat4 projection;\n"
        "void main()\n"
        "{\n"
        "   FragPos = vec3(model * vec4(aPos, 1.0));\n"
        "   Normal = mat3(transpose(inverse(model))) * aNormal;\n"
        "   gl_Position = projection * view * vec4(FragPos, 1.0);\n"
        "}\0";
    
    // 프래그먼트 셰이더 소스
    const char* fragmentShaderSource = "#version 330 core\n"
        "out vec4 FragColor;\n"
        "in vec3 Normal;\n"
        "in vec3 FragPos;\n"
        "uniform vec4 meshColor;\n"
        "uniform float opacity;\n"
        "void main()\n"
        "{\n"
        "   vec3 lightPos = vec3(5.0, 5.0, 5.0);\n"
        "   vec3 norm = normalize(Normal);\n"
        "   vec3 lightDir = normalize(lightPos - FragPos);\n"
        "   float diff = max(dot(norm, lightDir), 0.0);\n"
        "   vec3 diffuse = diff * vec3(1.0, 1.0, 1.0);\n"
        "   vec3 ambient = vec3(0.2, 0.2, 0.2);\n"
        "   vec3 result = (ambient + diffuse) * vec3(meshColor);\n"
        "   FragColor = vec4(result, opacity);\n"
        "}\0";
    
    // 셰이더 프로그램 컴파일
    GLuint shaderProgram = compileShader(vertexShaderSource, fragmentShaderSource);
    
    // 모델 로드 - 파일명 기본값 설정
    // const char* originalModelPath = "beshon.obj";
    const char* convexObjPath = "decomp.obj";
    // const char* convexStlPath = "decomp.stl";
    
    // 명령줄 인수가 있으면 파일명 대체
    // if (argc > 1) originalModelPath = argv[1];
    // if (argc > 2) convexObjPath = argv[2];
    // if (argc > 3) convexStlPath = argv[3];

    std::vector<Mesh> convexParts = loadOBJPart(convexObjPath);
    
    // 각 파트에 대해 처리
    for (auto& mesh : convexParts) {
        centerAndScaleModel(mesh);
        mesh.setupMesh();
    }

    // 모델 생성
    // Mesh originalModel(glm::vec4(0.0f, 0.0f, 1.0f, 1.0f)); // 파란색
    Mesh convexObjModel(glm::vec4(1.0f, 0.0f, 0.0f, 0.5f)); // 반투명 빨간색
    // Mesh convexStlModel(glm::vec4(0.0f, 1.0f, 0.0f, 0.5f)); // 반투명 초록색
    
    // 모델 로드
    // bool originalLoaded = loadOBJ(originalModelPath, originalModel);
    bool convexObjLoaded = loadOBJ(convexObjPath, convexObjModel);
    // bool convexStlLoaded = loadSTL(convexStlPath, convexStlModel);
    
    // 모델 중앙 정렬 및 크기 조정
    // if (originalLoaded) centerAndScaleModel(originalModel);
    // if (convexObjLoaded) centerAndScaleModel(convexObjModel);
    // if (convexStlLoaded) centerAndScaleModel(convexStlModel);

    // 모델 정보 출력
    // if (originalLoaded) printModelInfo(originalModel);
    // if (convexObjLoaded) printModelInfo(convexObjModel);
    // if (convexStlLoaded) printModelInfo(convexStlModel);

    // 메시 설정
    // if (originalLoaded) originalModel.setupMesh();
    // if (convexObjLoaded) convexObjModel.setupMesh();
    // if (convexStlLoaded) convexStlModel.setupMesh();
    
    
    // 렌더링 루프
    while (!glfwWindowShouldClose(window)) {
        // 델타 타임 계산
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        
        // 궤도 카메라
        updateCameraPosition();

        // 입력 처리
        processInput(window);
        
        // 렌더링
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // 와이어프레임 모드 설정
        if (wireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        } else {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
        
        // 셰이더 사용
        glUseProgram(shaderProgram);
        
        // 투영 행렬 설정
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        
        // 뷰 행렬 설정
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        
        // 모델 행렬 설정
        glm::mat4 model = glm::mat4(1.0f);
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        
        // 모델 렌더링
        // if (originalLoaded) {
        //     originalModel.draw(shaderProgram);
        // }
        
        if (convexObjLoaded) {
            // convexObjModel.draw(shaderProgram);
            for (const auto& mesh : convexParts) {
                mesh.draw(shaderProgram);
            }
        }
        
        
        // if (convexStlLoaded) {
        //     convexStlModel.draw(shaderProgram);
        // }
        
        // 버퍼 교체 및 이벤트 폴링
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    // 정리
    glfwTerminate();
    return 0;
}