# 最低版本限制
cmake_minimum_required(VERSION 3.24)

# 项目名称
project("cmsis_dap" C ASM)

# 寻找源文件
file(GLOB SOURCE_FILES "*.c")

# 添加源码
add_library(${PROJECT_NAME}
    ${SOURCE_FILES}
)

# 链接库
target_link_libraries(${PROJECT_NAME}
PUBLIC
    "CMSIS"
    "gd32f4_std"
    "ulog"
    "misc"
)

# 头文件路径
target_include_directories(${PROJECT_NAME}
PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
