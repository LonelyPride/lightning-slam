add_executable(run_frontend_offline
        run_frontend_offline.cc
)

target_link_libraries(run_frontend_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loop_offline
        run_loop_offline.cc
)

target_link_libraries(run_loop_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_slam_offline
        run_slam_offline.cc
)

target_link_libraries(run_slam_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_slam_online
        run_slam_online.cc
)

target_link_libraries(run_slam_online
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loc_offline
        run_loc_offline.cc
)

target_link_libraries(run_loc_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loc_online
        run_loc_online.cc
)

target_link_libraries(run_loc_online
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

# 个人修改（2026-10-09，aarch64 交叉编译支持）：test_ui 直接 include <pangolin/...>，
# 只有编真 UI 的构建才有它（交叉编译默认 LIGHTNING_WITH_UI=OFF）。
if (LIGHTNING_WITH_UI)
    add_executable(test_ui
            test_ui.cc
    )

    target_link_libraries(test_ui
            ${PROJECT_NAME}.libs
            ${third_party_libs}
    )
else ()
    message(STATUS "${PROJECT_NAME}: LIGHTNING_WITH_UI=OFF -> skip test_ui")
endif ()

install(TARGETS run_slam_offline run_slam_online run_loc_offline run_loc_online
        DESTINATION lib/${PROJECT_NAME})

# 个人修改（2026-10-02，方案C）：姿态/重力观测自检程序（纯算法自检，不需要数据/ROS 话题）
#   验证"重力作为加计观测"的符号约定/维度/权重/步长是否正确：把姿态摆歪 20° 后应单调收敛回竖直
add_executable(test_gravity_obs
        test_gravity_obs.cc
)
target_link_libraries(test_gravity_obs
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)
