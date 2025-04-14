cmake_minimum_required(VERSION 3.10)
project(3d_collision_detection)

# C++11 표준 사용
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 소스 파일 목록
file(GLOB_RECURSE MATH_SOURCES "src/math/*.cpp")
file(GLOB_RECURSE GEOMETRY_SOURCES "src/geometry/*.cpp")
file(GLOB_RECURSE CORE_SOURCES "src/core/*.cpp")
file(GLOB_RECURSE DECOMPOSITION_SOURCES "src/decomposition/*.cpp")
file(GLOB_RECURSE COLLISION_SOURCES "src/collision/*.cpp")

# 헤더 파일 포함 경로
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/core
    ${CMAKE_SOURCE_DIR}/include/math
    ${CMAKE_SOURCE_DIR}/include/geometry
    ${CMAKE_SOURCE_DIR}/include/collision
    ${CMAKE_SOURCE_DIR}/include/decomposition
)

# 테스트 실행 파일 대상 추가
add_executable(collision_test
    src/test.cpp
    ${MATH_SOURCES}
    ${GEOMETRY_SOURCES}
    ${CORE_SOURCES}
    ${DECOMPOSITION_SOURCES}
    ${COLLISION_SOURCES}
)

# V-HACD 의존성 추가 (해당 부분은 V-HACD가 어떻게 빌드되는지에 따라 조정 필요)
# 만약 V-HACD가 이미 빌드되어 있고 링크만 필요한 경우:
# target_link_libraries(collision_test PRIVATE ${CMAKE_SOURCE_DIR}/vhacd/app/build/TestVHACD)

# 또는 V-HACD를 직접 빌드하려면 (하위 프로젝트로 추가):
# add_subdirectory(vhacd/app)
# target_link_libraries(collision_test PRIVATE TestVHACD)

# OpenMP 찾기 및 링크 (멀티스레딩을 위해)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(collision_test PRIVATE OpenMP::OpenMP_CXX)
endif()

# 필요한 경우 더 많은 라이브러리 링크 (예: 수학 라이브러리)
target_link_libraries(collision_test PRIVATE m)

# 출력 파일 위치 지정
# set_target_properties(collision_test PROPERTIES)
#    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
# )

# 테스트 데이터 복사 (OBJ 파일 등)
# file(COPY ${CMAKE_SOURCE_DIR}/vhacd/app/meshes/lamp.obj
#     DESTINATION ${CMAKE_BINARY_DIR}/bin)