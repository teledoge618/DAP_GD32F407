# 最低版本限制
cmake_minimum_required(VERSION 3.24)

# 引入自定义CMAKE函数
include("${CMAKE_SOURCE_DIR}/scripts/cmake/set_target_linker.cmake")
include("${CMAKE_SOURCE_DIR}/scripts/cmake/generate_hex.cmake")
include("${CMAKE_SOURCE_DIR}/scripts/cmake/generate_bin.cmake")
include("${CMAKE_SOURCE_DIR}/scripts/cmake/print_all_variables.cmake")

# 项目名称
project("app" C)

# 寻找源文件
file(GLOB SOURCE_FILES "*.c")

# 添加源码
add_executable(${PROJECT_NAME}
    ${SOURCE_FILES}
)

# 链接库
target_link_libraries(${PROJECT_NAME}
PRIVATE
    "CMSIS"
    "gd32f4_std"
    "ulog"
    "misc"
    "cmsis_dap"
    "cherry_usb"
)

# 头文件路径
target_include_directories(${PROJECT_NAME}
PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

# 添加链接脚本
set_target_linker(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/scripts/ldscripts/GD32F407VE_FLASH_APP.ld)

# 二进制文件生成
# generate_binary(${PROJECT_NAME})

# 打印
# print_all_variables()

generate_hex()
generate_bin()