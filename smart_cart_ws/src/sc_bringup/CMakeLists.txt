cmake_minimum_required(VERSION 3.8)
project(sc_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ── 리소스 설치 (중요) ──
# 여기서 maps 폴더를 추가해야 맵 토픽을 만들 때 파일을 불러올 수 있습니다.
install(DIRECTORY config launch maps rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()