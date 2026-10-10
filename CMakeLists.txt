cmake_minimum_required(VERSION 3.16)

project(lightning)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# set to release by default
set(CMAKE_BUILD_TYPE Release)

# -------------------------------------------------------------
# 导入第三方库，同时做一些全局编译选项配置
add_definitions("-DPCL_NO_PRECOMPILE")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp -fPIC -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O2 -fopenmp -fPIC -g -ggdb")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -ggdb -g -fopenmp -fPIC -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -g")


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# 本机构建时把可执行文件写到源码树 bin/（仓库里跟踪的 x86-64 调试二进制）。
# 个人修改（2026-10-09，aarch64 交叉编译支持）：交叉编译产出的 aarch64 ELF 不能覆盖它们，
# 因此交叉构建只输出到 build 目录，由 src/app 的 install(TARGETS ...) 负责拷贝到
# install/lib/lightning。详见 doc/cross_compile_aarch64.md。
if (NOT CMAKE_CROSSCOMPILING)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
endif ()
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/packages.cmake)

# msg 和 srv
set(MY_SRV_FILES "")
set(SRC_DIR ${PROJECT_SOURCE_DIR}/srv)
file(GLOB_RECURSE SRC_FILES ${SRC_DIR}/*.srv)
foreach (SRC_FILE ${SRC_FILES})
    file(RELATIVE_PATH REL_PATH ${SRC_DIR} ${SRC_FILE})
    list(APPEND MY_SRV_FILES ${PROJECT_SOURCE_DIR}:srv/${REL_PATH})
endforeach ()

rosidl_generate_interfaces(${PROJECT_NAME}
        ${MY_SRV_FILES}
        ${MY_MSG_FILES}
        ${MY_ACTION_FILES}
        DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs
)

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

install(DIRECTORY config
        DESTINATION lib/${PROJECT_NAME}/
)

# 个人修改（2026-10-09，map_manager_3d 对接）：把 launch/ 装到 share/lightning/launch，
# 让 <pkg> 形式的 launch 命令可用（ros2 launch 会在包 share 目录里递归查找 launch 文件）：
#     ros2 launch lightning loc_online.launch.py  config:=<loc_*.yaml>
#     ros2 launch lightning slam_online.launch.py config:=<slam.yaml>
# map_manager_3d 的 map_manager_3d_node.cpp::spawn_mode() 就是拼这两条命令启动建图/定位进程的。
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

add_subdirectory(thirdparty/livox_ros_driver)

add_subdirectory(src)

# 个人修改（2026-10-09，aarch64 交叉编译支持）：把常用 python 小工具一起装进包里，
# 上板后可在 $(ros2 pkg prefix lightning)/lib/lightning/ 下直接调用。
install(PROGRAMS scripts/nav_manager_map_export.py
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
