# 最低版本限制
cmake_minimum_required(VERSION 3.24)

MESSAGE(STATUS "using toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE) # 编译时输出命令
add_definitions(-DGD32F407)             # 添加全局宏定义

add_compile_definitions(
	$<$<CONFIG:Debug>:DEBUG>	# 自动添加DEBUG宏定义
)

# 目标
project("dap_gd32f407")

# 添加子文件夹
add_subdirectory("source")

