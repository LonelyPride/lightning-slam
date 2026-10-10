# aarch64 交叉编译（速查入口）

完整文档（前置条件、参数表、路径重定位原理、上板部署、排障、交叉环境维护）见：

**[`doc/cross_compile_aarch64.md`](../doc/cross_compile_aarch64.md)**

最短路径（本工作区：`/home/h/ddh/code/ws/lightning_ws`，交叉环境：`/home/h/ddh/code/ws/cross-aarch64`）：

```bash
# 开发机
./src/lightning-slam/scripts/build_aarch64.sh --deploy dist_aarch64 \
  --deploy-prefix /home/linux/ddh/code/lightning_ws/install
rsync -a --delete dist_aarch64/ <user>@<board>:/home/linux/ddh/code/lightning_ws/install/

# 主板
source ~/ddh/code/lightning_ws/install/setup.bash
ros2 pkg prefix lightning
ros2 run lightning run_loc_online --config $(ros2 pkg prefix lightning)/lib/lightning/config/gaoda.yaml
```

两个脚本：

| 脚本 | 作用 |
|---|---|
| `build_aarch64.sh` | 交叉编译入口（`--clean` / `--deploy DIR` / `--relocate` / `--march` / `--jobs` / `--cross-root` / `--ui` / `--help`） |
| `relocate_install_tree.py` | 安装树路径重定位（被 `build_aarch64.sh` 自动调用，也可单独执行） |

> 交叉编译的 3 个改动点（缺一个就会失败）：`cmake/packages.cmake` 交叉编译用
> `-march=armv8-a` 并把 Pangolin 变成可选、根 `CMakeLists.txt` 交叉编译不写源码树
> `bin/`、`src/ui/pangolin_window.h` 提供无 Pangolin 时的空实现。
