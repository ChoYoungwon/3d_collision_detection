cmake_minimum_required(VERSION 3.10)
project(3D_Collision_Detection)

# C++11 표준 사용
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenMP 찾기
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# 소스 파일 목록
file(GLOB_RECURSE SOURCES "src/*.cpp")

# 헤더 파일 디렉토리
include_directories(include)

# V-HACD 라이브러리 설정 (라이브러리 경로 조정 필요)
# add_subdirectory(lib/v-hacd)
# include_directories(lib/v-hacd/include)

# 실행 파일 빌드
add_executable(collision_detection ${SOURCES})

# 라이브러리 링크
# target_link_libraries(collision_detection vhacd)