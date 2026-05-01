cmake_minimum_required(VERSION 3.8)
project(sc_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# config / launch / maps / urdf / rviz / worlds 설치
install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# 선택 폴더 (있으면 설치)
foreach(dir rviz urdf worlds)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

# OpenCR 펌웨어 폴더 설치 (smart_cart_ws/firmware → share)
# ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware 에서 빌드 시점에 복사
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/
    DESTINATION share/${PROJECT_NAME}/firmware
  )
endif()

ament_package()
