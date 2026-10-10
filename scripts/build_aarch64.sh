#!/usr/bin/env bash
# 交叉编译 lightning-slam（ROS 包名 lightning）为 aarch64，供 NVIDIA Jetson 等 ARM 主板运行。
#
# 个人修改（2026-10-09，aarch64 交叉编译支持）：本脚本做法完全借鉴参考工程
#   /home/h/ddh/code/test/lidar-slam_ws/src/lidar-slam/scripts/build_aarch64.sh
# 复用仓库级交叉环境（默认工作区同级的 ../cross-aarch64，或 $CROSS_AARCH64_ROOT / --cross-root），
# 只构建 lightning 一个包，产物在仓库根：build_aarch64/ 与 install_aarch64/。
# 目标端依赖库、ROS Humble 全部来自交叉环境的 sysroot，不混用宿主 x86_64 库。
#
# 与本工程的差异（详见 doc/cross_compile_aarch64.md）：
#   * 本包带 Pangolin 3D UI，交叉环境的 sysroot 里没有 Pangolin → 交叉编译默认
#     -DLIGHTNING_WITH_UI=OFF（src/ui 不编，用 src/ui/pangolin_window.h 的空实现）。
#     确实要在交叉环境里带 UI 时用 --ui（需要 sysroot 里有 Pangolin）。
#   * Livox CustomMsg 支持（thirdparty/livox_ros_driver）随包一起交叉编译，默认保留。
#   * 产物是 run_loc_online / run_slam_online / run_loc_offline / run_slam_offline。
#
# 用法：
#   ./scripts/build_aarch64.sh                    # 构建
#   ./scripts/build_aarch64.sh --clean            # 清理后重新构建
#   ./scripts/build_aarch64.sh --march armv8.2-a  # 针对 Xavier/Orin 优化
#   ./scripts/build_aarch64.sh --deploy dist_aarch64   # 直接产出可上板的安装树
#
# 注意：install_aarch64/ 里保留了构建机的绝对路径（sysroot ROS underlay、install 前缀），
# 直接 rsync 到板子上 source 会报 not found。上板前请用 --deploy（推荐）或 --relocate。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${HERE}/.." && pwd)"

# 向上定位工作区根：要求 <root>/src/ 正好包含本包，roboctl 集成树与独立
# lightning ws 都适用（包自身也有 src/，所以必须用包含关系判断）。
ROOT="${PKG_DIR}"
while [[ "${ROOT}" != "/" ]]; do
  ROOT="$(cd "${ROOT}/.." && pwd)"
  if [[ -d "${ROOT}/src" && "${PKG_DIR}" == "${ROOT}/src/"* ]]; then
    break
  fi
done
if [[ ! -d "${ROOT}/src" || "${PKG_DIR}" != "${ROOT}/src/"* ]]; then
  echo "error: workspace root not found above ${PKG_DIR}" >&2
  exit 1
fi

# 交叉环境：--cross-root > $CROSS_AARCH64_ROOT > 工作区同级 ../cross-aarch64
#            > 工作区上上级 ../../cross-aarch64 > $HOME/code/test/cross-aarch64
# 逐个候选检查 env.sh + sysroot，第一个可用的胜出（缺 env.sh 的残树会被跳过）。
CROSS_ROOT_OPT=""
CROSS_ROOT=""
pick_cross_root() {
  local candidate
  for candidate in "$@"; do
    [[ -n "${candidate}" ]] || continue
    if [[ -f "${candidate}/env.sh" && -e "${candidate}/sysroot/etc/os-release" ]]; then
      (cd "${candidate}" && pwd)
      return 0
    fi
  done
  return 1
}
if [[ -n "${CROSS_AARCH64_ROOT:-}" ]]; then
  CROSS_ROOT="${CROSS_AARCH64_ROOT}"
elif [[ -n "${CROSS_ROOT_OPT:-}" ]]; then
  CROSS_ROOT="${CROSS_ROOT_OPT}"
else
  CROSS_ROOT="$(pick_cross_root \
    "$(cd "${ROOT}/.." && pwd)/cross-aarch64" \
    "$(cd "${ROOT}/../.." && pwd)/cross-aarch64" \
    "${HOME}/code/test/cross-aarch64" \
    "${HOME}/code/ws/cross-aarch64" \
    "${HOME}/cross-aarch64" || true)"
fi

BUILD_BASE="${ROOT}/build_aarch64"
INSTALL_BASE="${ROOT}/install_aarch64"
LOG_BASE="${ROOT}/log_aarch64"
MARCH="${LIGHTNING_ARM_MARCH:-armv8-a}"
JOBS="${LIGHTNING_CROSS_JOBS:-4}"
LOAD_LIMIT="${LIGHTNING_CROSS_LOAD_LIMIT:-24}"
CLEAN=0
# 上板相关：install 树里是构建机路径，需要重定位后才能 source。
RELOCATE=0
RELOCATED=0
DEPLOY_DIR=""
DEPLOY_PREFIX=""
ROS_PREFIX="${LIGHTNING_ROS_PREFIX:-/opt/ros/humble}"
# UI：默认让 CMake 自己决定（交叉编译 UI=OFF）；--ui / --no-ui 可覆盖
UI_ARG=""

usage() {
  cat <<'EOF'
Usage: scripts/build_aarch64.sh [options]

  Cross-compile the lightning-slam package (ROS name: lightning) for aarch64.

Options:
  --cross-root DIR   cross environment root (default: ../cross-aarch64 next to the workspace,
                     or $CROSS_AARCH64_ROOT)
  --march ARCH       aarch64 -march baseline (default: armv8-a; Xavier/Orin: armv8.2-a)
  --jobs N           compiler jobs (default: 4, env LIGHTNING_CROSS_JOBS)
  --ui               build the Pangolin 3D UI as well (-DLIGHTNING_WITH_UI=ON);
                     the cross sysroot must provide Pangolin, otherwise configure fails
  --no-ui            explicitly disable the Pangolin 3D UI (default when cross-compiling)
  --clean            remove build_aarch64/ install_aarch64/ log_aarch64/ first
  -h, --help         show this help

Packaging / board deploy:
  --deploy DIR       after build, copy install_aarch64/ to DIR, rewrite the build-host
                     paths in it and leave it ready to rsync (recommended)
  --deploy-prefix P  where DIR will live on the board (default: DIR; baked into the tree)
  --relocate         rewrite install_aarch64/ in place instead of producing a copy
  --ros-prefix P     ROS prefix on the board (default: /opt/ros/humble)

Environment:
  CROSS_AARCH64_ROOT          alternative way to set the cross environment root
  LIGHTNING_ARM_MARCH         aarch64 -march baseline
  LIGHTNING_CROSS_JOBS        compiler jobs (default: 4)
  LIGHTNING_CROSS_LOAD_LIMIT  make load limit (default: 24)
  LIGHTNING_ROS_PREFIX        board ROS prefix (default: /opt/ros/humble)

Outputs:
  <workspace>/install_aarch64/lib/lightning/run_loc_online   (aarch64 ELF)
  <workspace>/install_aarch64/lib/liblightning.libs.so
EOF
  cat <<EOF

Detected:
  package:   ${PKG_DIR}
  workspace: ${ROOT}
  cross env: ${CROSS_ROOT:-<not found: use --cross-root>}
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cross-root|--cross-root=*)
      if [[ "$1" == *=* ]]; then CROSS_ROOT_OPT="${1#*=}"; else CROSS_ROOT_OPT="${2:?--cross-root requires a value}"; shift; fi
      ;;
    --march|--march=*)
      if [[ "$1" == *=* ]]; then MARCH="${1#*=}"; else MARCH="${2:?--march requires a value}"; shift; fi
      ;;
    --jobs|--jobs=*)
      if [[ "$1" == *=* ]]; then JOBS="${1#*=}"; else JOBS="${2:?--jobs requires a value}"; shift; fi
      ;;
    --ui) UI_ARG="-DLIGHTNING_WITH_UI=ON" ;;
    --no-ui) UI_ARG="-DLIGHTNING_WITH_UI=OFF" ;;
    --clean) CLEAN=1 ;;
    --relocate) RELOCATE=1 ;;
    --deploy|--deploy=*)
      if [[ "$1" == *=* ]]; then DEPLOY_DIR="${1#*=}"; else DEPLOY_DIR="${2:?--deploy requires a value}"; shift; fi
      ;;
    --deploy-prefix|--deploy-prefix=*)
      if [[ "$1" == *=* ]]; then DEPLOY_PREFIX="${1#*=}"; else DEPLOY_PREFIX="${2:?--deploy-prefix requires a value}"; shift; fi
      ;;
    --ros-prefix|--ros-prefix=*)
      if [[ "$1" == *=* ]]; then ROS_PREFIX="${1#*=}"; else ROS_PREFIX="${2:?--ros-prefix requires a value}"; shift; fi
      ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "unknown argument: $1" ;;
  esac
  shift
done

[[ -n "${CROSS_ROOT_OPT}" ]] && CROSS_ROOT="${CROSS_ROOT_OPT}"

[[ -n "${CROSS_ROOT}" && -d "${CROSS_ROOT}" ]] ||
  die "cross root not found (use --cross-root or set CROSS_AARCH64_ROOT); expected e.g. $(cd "${ROOT}/.." && pwd)/cross-aarch64"
[[ -f "${CROSS_ROOT}/env.sh" ]] ||
  die "cross env not found: ${CROSS_ROOT}/env.sh"

# 提供交叉编译器、host 侧 ament/colcon，并把 sysroot 写入 AMENT/CMAKE_PREFIX_PATH。
# shellcheck source=/dev/null
source "${CROSS_ROOT}/env.sh"

command -v aarch64-linux-gnu-g++ >/dev/null ||
  die "aarch64 cross compiler missing (see ${CROSS_ROOT}/README.md)"
[[ -e "${CROSS_AARCH64_SYSROOT}/etc/os-release" ]] ||
  die "sysroot empty: ${CROSS_AARCH64_SYSROOT}"
[[ -d "${CROSS_AARCH64_SYSROOT}/opt/ros/humble" ]] ||
  die "ROS Humble missing in sysroot: ${CROSS_AARCH64_SYSROOT}/opt/ros/humble"

# 只让目标端前缀可见，避免宿主 /opt/ros/humble 或工作区里的 x86_64 结果被误用。
export AMENT_PREFIX_PATH="${CROSS_AARCH64_SYSROOT}/opt/ros/humble"
export CMAKE_PREFIX_PATH="${CROSS_AARCH64_SYSROOT}/opt/ros/humble"
export COLCON_PREFIX_PATH=""
export QEMU_LD_PREFIX="${CROSS_AARCH64_SYSROOT}"
export MAKEFLAGS="-j${JOBS} -l${LOAD_LIMIT}"

cd "${ROOT}"
if [[ "${CLEAN}" -eq 1 ]]; then
  rm -rf "${BUILD_BASE}" "${INSTALL_BASE}" "${LOG_BASE}"
fi

echo "cross-compile: lightning (lightning-slam) -> aarch64"
echo "  cross root: ${CROSS_ROOT}"
echo "  sysroot:    ${CROSS_AARCH64_SYSROOT}"
echo "  compiler:   $(command -v aarch64-linux-gnu-g++)"
echo "  march:      ${MARCH}  (host arch: $(uname -m))"
echo "  UI:         ${UI_ARG:-<CMake 默认：交叉编译 OFF>}"
echo "  install:    ${INSTALL_BASE}"

# roboctl 集成树提供 find_library 的 sysroot 兜底；独立 lightning ws 没有该文件，
# 存在才注入（CMAKE_PROJECT_INCLUDE_BEFORE 指向不存在的文件会让 project() 直接失败）。
CMAKE_EXTRA=()
if [[ -f "${ROOT}/scripts/cmake/find_library_fallback.cmake" ]]; then
  CMAKE_EXTRA+=("-DCMAKE_PROJECT_INCLUDE_BEFORE=${ROOT}/scripts/cmake/find_library_fallback.cmake")
fi
[[ -n "${UI_ARG}" ]] && CMAKE_EXTRA+=("${UI_ARG}")

colcon --log-base "${LOG_BASE}" build \
  --base-paths "${PKG_DIR}" \
  --packages-up-to lightning \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --merge-install \
  --parallel-workers "${JOBS}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_TOOLCHAIN_FILE="${CROSS_AARCH64_CMAKE_TOOLCHAIN}" \
    -DCROSS_AARCH64_SYSROOT="${CROSS_AARCH64_SYSROOT}" \
    -DCMAKE_FIND_ROOT_PATH="${CROSS_AARCH64_SYSROOT};${CROSS_AARCH64_SYSROOT}/usr;${CROSS_AARCH64_SYSROOT}/opt/ros/humble;${INSTALL_BASE}" \
    ${CMAKE_EXTRA[@]+"${CMAKE_EXTRA[@]}"} \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPython3_LIBRARY="${CROSS_AARCH64_SYSROOT}/usr/lib/aarch64-linux-gnu/libpython3.10.so" \
    -DPython3_INCLUDE_DIR="${CROSS_AARCH64_SYSROOT}/usr/include/python3.10" \
    -DOPENNI_LIBRARY="${CROSS_AARCH64_SYSROOT}/usr/lib/libOpenNI.so" \
    -DOPENNI_INCLUDE_DIR="${CROSS_AARCH64_SYSROOT}/usr/include/ni" \
    -DZLIB_LIBRARY="${CROSS_AARCH64_SYSROOT}/usr/lib/aarch64-linux-gnu/libz.so" \
    -DZLIB_INCLUDE_DIR="${CROSS_AARCH64_SYSROOT}/usr/include" \
    -DBUILD_TESTING=OFF \
    -DLIGHTNING_ARM_MARCH="${MARCH}"

# 交叉编译最常见的错误是"产物其实还是 x86_64"，这里逐个校验 ELF 架构。
# readelf 的字段名会随 locale 翻译，因此固定用 LC_ALL=C 解析。
check_arch() {
  local bin="$1"
  [[ -f "${bin}" ]] || die "expected artifact missing: ${bin}"
  if command -v readelf >/dev/null; then
    local machine
    machine="$(LC_ALL=C readelf -h "${bin}" | awk -F: '/Machine:/{print $2}' | xargs)"
    [[ "${machine}" == "AArch64" ]] || die "artifact is not aarch64 (${machine:-unknown}): ${bin}"
  fi
}

for name in run_loc_online run_slam_online run_loc_offline run_slam_offline; do
  check_arch "${INSTALL_BASE}/lib/lightning/${name}"
done
check_arch "${INSTALL_BASE}/lib/liblightning.libs.so"
echo "verified: AArch64 ELF"


# ---- 上板准备：重写 install 树里的构建机路径 ----
RELOCATOR="${HERE}/relocate_install_tree.py"

relocate_tree() {
  local dir="$1"
  local -a args=(--install-dir "${dir}" --ros-prefix "${ROS_PREFIX}")
  if [[ -n "${DEPLOY_PREFIX}" ]]; then
    args+=(--new-prefix "${DEPLOY_PREFIX}")
  fi
  [[ -f "${RELOCATOR}" ]] || die "relocator not found: ${RELOCATOR}"
  python3 "${RELOCATOR}" "${args[@]}"
}

if [[ "${RELOCATE}" -eq 1 ]]; then
  echo
  echo "==> relocate in place: ${INSTALL_BASE}"
  relocate_tree "${INSTALL_BASE}"
  RELOCATED=1
fi

if [[ -n "${DEPLOY_DIR}" ]]; then
  echo
  mkdir -p "${DEPLOY_DIR}"
  DEPLOY_DIR="$(cd "${DEPLOY_DIR}" && pwd)"
  echo "==> stage deployable install tree: ${DEPLOY_DIR}"
  if command -v rsync >/dev/null; then
    rsync -a --delete "${INSTALL_BASE}/" "${DEPLOY_DIR}/"
  else
    rm -rf "${DEPLOY_DIR}"
    mkdir -p "${DEPLOY_DIR}"
    cp -a "${INSTALL_BASE}/." "${DEPLOY_DIR}/"
  fi
  [[ -n "${DEPLOY_PREFIX}" ]] || DEPLOY_PREFIX="${DEPLOY_DIR}"
  relocate_tree "${DEPLOY_DIR}"
  grep -q 'cross-aarch64/sysroot' "${DEPLOY_DIR}/setup.bash" 2>/dev/null &&
    die "staged tree still points at the build sysroot: ${DEPLOY_DIR}"
  check_arch "${DEPLOY_DIR}/lib/lightning/run_loc_online"
  echo "staged ok: ${DEPLOY_DIR}  (board path: ${DEPLOY_PREFIX})"
fi

echo
echo "done. aarch64 artifacts under: ${INSTALL_BASE}"
ls -1 "${INSTALL_BASE}/lib/lightning" 2>/dev/null | sed 's/^/  lib\/lightning\//'
echo
if [[ -n "${DEPLOY_DIR}" ]]; then
  cat <<EOF
下一步（上板）：
  rsync -a --delete ${DEPLOY_DIR}/ <user>@<board>:<板上的 install 目录>/
  板上：source <板上的 install 目录>/setup.bash
        ros2 pkg prefix lightning      # 应指向该目录，再用 ros2 launch lightning ... / ros2 run lightning run_loc_online
  主板运行依赖：ROS Humble + libpcl / libopencv / libgoogle-glog / libyaml-cpp / libtbb；
  另注意 config/*.yaml 的 system.map_path / system.map_root_path 需改成主板上的实际路径。
  本包交叉编译默认不带 Pangolin 3D UI（system.with_ui 失效），定位/建图本身不受影响。
EOF
elif [[ "${RELOCATED}" -eq 1 ]]; then
  cat <<EOF
下一步（上板）：
  install_aarch64/ 已重定位，可直接 rsync：
  rsync -a --delete ${INSTALL_BASE}/ <user>@<board>:<板上的 install 目录>/
  板上：source <板上的 install 目录>/setup.bash
        ros2 pkg prefix lightning
EOF
else
  cat <<EOF
下一步（上板）：
  install_aarch64/ 里仍是构建机路径，直接 rsync 会让板上 source 报
  not found: "${CROSS_AARCH64_SYSROOT}/opt/ros/humble/local_setup.bash"。
  上板前二选一：
    python3 ${HERE}/relocate_install_tree.py --install-dir ${INSTALL_BASE}
    或本次构建加 --deploy DIR（推荐：产出一份已重定位、可直接 rsync 的副本）
EOF
fi

