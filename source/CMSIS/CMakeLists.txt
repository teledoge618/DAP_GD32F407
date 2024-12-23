# 最低版本限制
cmake_minimum_required(VERSION 3.24)

# 项目名称
project("CMSIS" C ASM)

# 添加源码
add_library(${PROJECT_NAME}
    "${CMAKE_CURRENT_SOURCE_DIR}/GD/GD32F4xx/source/GNU/startup_gd32f407xx.s"
)

# 头文件路径
target_include_directories(${PROJECT_NAME}
PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/core/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/GD/GD32F4xx/include"
)
