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
    
    // 모델 로드 - 파일명
    const char* ModelPath;
    
    // 명령줄 인수 확인
    if (argc < 3) {
        std::cout << "Usage : ./3d_viewer [num] [path]" << std::endl;
        std::cout << "num(1) : original obj" << std::endl;
        std::cout << "num(2) : decomp.obj" << std::endl;
        glfwTerminate();
        return -1;
    }
    
    ModelPath = argv[2];
    bool ObjLoaded = false;
    bool convexObjLoaded = false;
    Mesh ObjModel;
    std::vector<Mesh> convexParts;
    
    // 입력에 따라 다른 모델 로드
    if (atoi(argv[1]) == 2) {
        // 볼록화된 부분 로드
        convexParts = loadOBJPart(ModelPath);
        convexObjLoaded = !convexParts.empty();
        
        // 각 파트에 대해 처리
        for (auto& mesh : convexParts) {
            centerAndScaleModel(mesh);
            mesh.setupMesh();
        }
    } else {
        // 원본 모델 생성
        ObjModel = Mesh(glm::vec4(1.0f, 0.0f, 0.0f, 0.5f)); // 반투명 빨간색
        
        // 모델 로드
        ObjLoaded = loadOBJ(ModelPath, ObjModel);
        
        if (ObjLoaded) {
            // 모델 중앙 정렬 및 크기 조정
            centerAndScaleModel(ObjModel);
            
            // 모델 정보 출력
            printModelInfo(ObjModel);
            
            // 메시 설정
            ObjModel.setupMesh();
        }
    }
    
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
        if (convexObjLoaded) {
            for (const auto& mesh : convexParts) {
                mesh.draw(shaderProgram);
            }
        }
        
        if (ObjLoaded) {
            ObjModel.draw(shaderProgram);
        }
        
        // 버퍼 교체 및 이벤트 폴링
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    // 정리
    glfwTerminate();
    return 0;
}