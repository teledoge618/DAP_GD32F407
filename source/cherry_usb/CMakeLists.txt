# 最低版本限制
cmake_minimum_required(VERSION 3.24)

# 项目名称
project("cherry_usb" C)

# 添加源码
add_library(${PROJECT_NAME}
    "${CMAKE_CURRENT_SOURCE_DIR}/core/usbd_core.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/class/cdc/usbd_cdc_acm.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/port/dwc2/usb_dc_dwc2.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/port/dwc2/usb_glue_gd.c"
)

# 链接库
target_link_libraries(${PROJECT_NAME}
PUBLIC
    "CMSIS"
    "ulog"
    "misc"
)

# 头文件路径
target_include_directories(${PROJECT_NAME}
PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/common"
    "${CMAKE_CURRENT_SOURCE_DIR}/core"
    "${CMAKE_CURRENT_SOURCE_DIR}/class/cdc"
    "${CMAKE_CURRENT_SOURCE_DIR}/port/dwc2"
)
