﻿cmake_minimum_required (VERSION 3.24)

project ("ulog" C)

file(GLOB SOURCE_FILES "*.c")

add_library(${PROJECT_NAME} STATIC
	${SOURCE_FILES}
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
	"printf"
)

target_include_directories(${PROJECT_NAME}
PUBLIC
	"${CMAKE_CURRENT_SOURCE_DIR}"
)
