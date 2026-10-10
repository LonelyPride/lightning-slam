find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -----------------------------------------------------------------------------
# 个人修改（2026-10-09，aarch64 交叉编译支持）：可裁剪的功能开关
#   背景：工作区同级的 cross-aarch64 交叉环境（sysroot）里【没有 Pangolin】，
#         而 Pangolin 只用于 3D 可视化界面，上板定位（viz:=false）用不到。
#
#   LIGHTNING_WITH_UI
#     1 = 编译 src/ui 下的真 Pangolin 界面（本机构建默认 1，行为与改动前完全一致）
#     0 = 不编译 src/ui，改用 src/ui/pangolin_window.h 里的"空实现"
#         （交叉编译默认 0）。两边接口完全一致，src/ 里所有 `ui_->Xxx()`
#         调用点（本来就写在 `if (ui_)` 里）一行都不用改。
#   想在有 Pangolin 的交叉环境里带上 UI：-DLIGHTNING_WITH_UI=ON（构建脚本 --ui）
# -----------------------------------------------------------------------------
if (CMAKE_CROSSCOMPILING)
    set(_lightning_with_ui_default OFF)
else ()
    set(_lightning_with_ui_default ON)
endif ()
option(LIGHTNING_WITH_UI "编译 Pangolin 3D UI（交叉编译默认 OFF，需要 Pangolin）" ${_lightning_with_ui_default})

if (LIGHTNING_WITH_UI)
    find_package(Pangolin REQUIRED)
    find_package(OpenGL REQUIRED)
    add_compile_definitions(LIGHTNING_WITH_UI=1)
else ()
    add_compile_definitions(LIGHTNING_WITH_UI=0)
    message(STATUS "LIGHTNING_WITH_UI=OFF: build without Pangolin 3D UI (src/ui 被跳过，用空实现)")
endif ()

# OMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

# -march=native 只能用于"本机编译"：交叉编译器无法探测 x86 宿主的 CPU，
# 会直接报错 unknown value 'native' for '-march'。因此交叉编译时固定使用一个
# 向后兼容的基线（Jetson 上均可运行），需要针对具体板卡优化时用
#   -DLIGHTNING_ARM_MARCH=armv8.2-a   (Xavier / Orin)
# 覆盖；本机 arm / x86 构建行为保持不变。
set(LIGHTNING_ARM_MARCH "armv8-a" CACHE STRING
    "aarch64 cross build -march baseline (empty = compiler default)")

if (CMAKE_CROSSCOMPILING AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|armv7l")
    if (LIGHTNING_ARM_MARCH)
        add_compile_options(-march=${LIGHTNING_ARM_MARCH})
    endif ()
elseif (BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native)
else ()
    # 添加架构判断
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|armv7l")
        add_compile_options(-march=native)  # ARM 架构使用 native
    else()
        add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
    endif()
endif ()

# 只有编 UI 时才需要 Pangolin / GLEW 的头文件
if (LIGHTNING_WITH_UI)
    include_directories(
            ${Pangolin_INCLUDE_DIRS}
            ${GLEW_INCLUDE_DIRS}
    )
endif ()

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${tf2_INCLUDE_DIRS}
        ${pcl_conversions_INCLUDR_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
)

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/livox_ros_driver/rosidl_generator_cpp
)

include_directories(
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/thirdparty
)


set(third_party_libs
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        glog gflags
        ${yaml-cpp_LIBRARIES}
        ${pcl_conversions_LIBRARIES}
        tbb
        ${rosbag2_cpp_LIBRARIES}
)

# 只有编 UI 时才链接 Pangolin
if (LIGHTNING_WITH_UI)
    list(APPEND third_party_libs ${Pangolin_LIBRARIES})
endif ()

