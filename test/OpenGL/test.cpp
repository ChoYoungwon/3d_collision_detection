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
    
    void draw(GLuint shaderProgram) {
        glUniform4fv(glGetUniformLocation(shaderProgram, "meshColor"), 1, glm::value_ptr(color));
        glUniform1f(glGetUniformLocation(shaderProgram, "opacity"), opacity);
        
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
};

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
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
            // 기존 면 처리 코드와 동일
            std::string vertex1, vertex2, vertex3;
            iss >> vertex1 >> vertex2 >> vertex3;
            
            // 정점/텍스처/법선 인덱스 파싱
            std::replace(vertex1.begin(), vertex1.end(), '/', ' ');
            std::replace(vertex2.begin(), vertex2.end(), '/', ' ');
            std::replace(vertex3.begin(), vertex3.end(), '/', ' ');
            
            std::istringstream v1(vertex1), v2(vertex2), v3(vertex3);
            unsigned int vertexIndex, normalIndex;
            unsigned int temp;
            
            // 첫 번째 정점
            v1 >> vertexIndex;
            if (v1.peek() == ' ') v1 >> temp;
            v1 >> normalIndex;
            
            Vertex meshVertex;
            meshVertex.position = temp_vertices[vertexIndex-1];
            if (normalIndex-1 < temp_normals.size())
                meshVertex.normal = temp_normals[normalIndex-1];
            else
                meshVertex.normal = glm::vec3(0.0f, 0.0f, 1.0f);
                
            currentMesh.vertices.push_back(meshVertex);
            currentMesh.indices.push_back(currentMesh.vertices.size()-1);
            
            // 두 번째 정점
            v2 >> vertexIndex;
            if (v2.peek() == ' ') v2 >> temp;
            v2 >> normalIndex;
            
            meshVertex.position = temp_vertices[vertexIndex-1];
            if (normalIndex-1 < temp_normals.size())
                meshVertex.normal = temp_normals[normalIndex-1];
            else
                meshVertex.normal = glm::vec3(0.0f, 0.0f, 1.0f);
                
            currentMesh.vertices.push_back(meshVertex);
            currentMesh.indices.push_back(currentMesh.vertices.size()-1);
            
            // 세 번째 정점
            v3 >> vertexIndex;
            if (v3.peek() == ' ') v3 >> temp;
            v3 >> normalIndex;
            
            meshVertex.position = temp_vertices[vertexIndex-1];
            if (normalIndex-1 < temp_normals.size())
                meshVertex.normal = temp_normals[normalIndex-1];
            else
                meshVertex.normal = glm::vec3(0.0f, 0.0f, 1.0f);
                
            currentMesh.vertices.push_back(meshVertex);
            currentMesh.indices.push_back(currentMesh.vertices.size()-1);
        }
    }
    
    // 마지막 메시 추가
    if (!currentMesh.vertices.empty()) {
        meshes.push_back(currentMesh);
    }
    
    std::cout << "총 " << meshes.size() << "개의 볼록 파트를 로드했습니다." << std::endl;
    
    return meshes;
}

int main(){
    const char* convexObjPath = "decomp.obj";
    std::vector<Mesh> convexParts = loadOBJPart(convexObjPath);
}