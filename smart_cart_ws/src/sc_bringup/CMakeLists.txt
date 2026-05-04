cmake_minimum_required(VERSION 3.8)
project(sc_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 1. 필수 리소스 폴더 설치 (behavior_trees 추가)
# 이 리스트에 포함된 폴더들은 빌드 시 install 폴더로 복사됩니다.
install(DIRECTORY 
  config 
  launch 
  maps 
  rviz 
  urdf 
  worlds 
  behavior_trees  # ★ Nav2 동작 트리 XML 파일들을 위해 반드시 필요
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# 2. OpenCR 펌웨어 폴더 설치
# 로봇 배포 시 펌웨어 바이너리를 함께 관리하기 위한 설정입니다.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/
    DESTINATION share/${PROJECT_NAME}/firmware
  )
endif()

# 3. 테스트 및 환경 설정
ament_package()