#!/usr/bin/env python3
"""把交叉编译产出的 aarch64 install 树改成能在目标板直接 source 的样子。

个人修改（2026-10-09，aarch64 交叉编译支持）：本脚本与参考工程 lidar-slam 的
scripts/relocate_install_tree.py 相同（只改了示例路径），由 build_aarch64.sh 自动调用，
也可以单独执行。

交叉编译时，ROS underlay 与 install 前缀都是开发机的绝对路径，直接 rsync 到板子上
会报：

    not found: "/home/.../cross-aarch64/sysroot/opt/ros/humble/local_setup.bash"

本脚本重写两类路径（**只改文本文件，绝不动 ELF/so**）：

    <...>/sysroot/opt/ros/humble   ->  --ros-prefix      (默认 /opt/ros/humble)
    开发机的 install 前缀          ->  --new-prefix      (默认 install 目录当前路径)

用法：

    # 开发机上，编译完、rsync 之前执行（推荐）
    python3 scripts/relocate_install_tree.py \
        --install-dir /home/h/ddh/code/ws/lightning_ws/install_aarch64

    # 已经拷到板子上，也可以在板上就地执行
    python3 relocate_install_tree.py --install-dir ~/ddh/code/lightning_ws/install

    # 只看会改哪些文件
    python3 scripts/relocate_install_tree.py --install-dir ... --dry-run
"""
import argparse
import os
import re
import sys

TEXT_EXT = {".bash", ".sh", ".zsh", ".ps1", ".psm1", ".dsv", ".cmake", ".txt", ".py", ".cfg"}
# ament_index 的 parent_prefix_path/ 下每个包一个无扩展名文件，需要按目录名识别。
EXTRA_DIRS = {"parent_prefix_path"}
SYSROOT_ROS_RE = re.compile(r"/[^\s\"';]*/sysroot/opt/ros/humble")
SYSROOT_ANY_RE = re.compile(r"/[^\s\"';]*/cross-aarch64/sysroot")
OLD_PREFIX_RE = re.compile(r'_colcon_prefix_sh_COLCON_CURRENT_PREFIX="([^"]+)"')


def read_text(path):
    """返回文件文本；ELF/二进制返回 None（保证不会破坏可执行文件）。"""
    try:
        with open(path, "rb") as fh:
            head = fh.read(65536)
    except OSError:
        return None
    if head.startswith(b"\x7fELF") or b"\x00" in head:
        return None
    try:
        return head.decode("utf-8")
    except UnicodeDecodeError:
        return None


def detect_old_prefix(install_dir):
    """从 local_setup.sh 里读出构建时的 install 前缀。"""
    for rel in ("local_setup.sh", "local_setup.bash", "share"):
        path = os.path.join(install_dir, rel)
        if rel == "share":
            continue
        if not os.path.isfile(path):
            continue
        text = read_text(path)
        if not text:
            continue
        match = OLD_PREFIX_RE.search(text)
        if match:
            return match.group(1)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--install-dir", required=True,
                        help="install 树根目录（含 setup.bash 的那一层）")
    parser.add_argument("--ros-prefix", default="/opt/ros/humble",
                        help="板子上 ROS 的安装前缀（默认 /opt/ros/humble）")
    parser.add_argument("--new-prefix", default=None,
                        help="install 树在板上的实际路径（默认 = --install-dir 的绝对路径）")
    parser.add_argument("--old-prefix", default=None,
                        help="构建时的 install 前缀（默认从 local_setup.sh 自动识别）")
    parser.add_argument("--dry-run", action="store_true", help="只报告，不写文件")
    args = parser.parse_args()

    install_dir = os.path.abspath(os.path.expanduser(args.install_dir))
    if not os.path.isfile(os.path.join(install_dir, "setup.bash")):
        print(f"error: {install_dir}/setup.bash not found; is this an install tree?", file=sys.stderr)
        return 1

    old_prefix = args.old_prefix or detect_old_prefix(install_dir)
    new_prefix = os.path.abspath(os.path.expanduser(args.new_prefix)) if args.new_prefix else install_dir
    ros_prefix = os.path.abspath(os.path.expanduser(args.ros_prefix))

    print(f"install dir: {install_dir}")
    print(f"ros prefix:  {ros_prefix}")
    if old_prefix:
        print(f"old prefix:  {old_prefix}")
        print(f"new prefix:  {new_prefix}")
    else:
        print("old prefix:  (not detected; only the sysroot ROS path will be rewritten)")

    changed = scanned = 0
    leftovers = []
    for root, _dirs, files in os.walk(install_dir):
        in_extra_dir = any(part in EXTRA_DIRS for part in root.split(os.sep))
        for name in sorted(files):
            path = os.path.join(root, name)
            if os.path.islink(path):
                continue
            ext = os.path.splitext(name)[1]
            if not in_extra_dir and ext not in TEXT_EXT:
                continue
            scanned += 1
            text = read_text(path)
            if text is None:
                continue
            updated = SYSROOT_ROS_RE.sub(ros_prefix, text)
            if old_prefix and old_prefix in updated:
                updated = updated.replace(old_prefix, new_prefix)
            if updated != text:
                changed += 1
                print(f"  rewrite {os.path.relpath(path, install_dir)}")
                if not args.dry_run:
                    with open(path, "w", encoding="utf-8") as fh:
                        fh.write(updated)
            elif SYSROOT_ANY_RE.search(updated):
                # 例如 python 头文件路径等：只影响基于本 install 再编译，不影响运行。
                leftovers.append(os.path.relpath(path, install_dir))

    print(f"scanned={scanned} changed={changed}" + (" (dry-run)" if args.dry_run else ""))
    if leftovers:
        print("注意：以下文件仍含开发机 sysroot 路径（仅影响二次编译，不影响运行）：")
        for item in leftovers:
            print(f"  {item}")
    if changed and not args.dry_run:
        print(f"\n现在可以 source {install_dir}/setup.bash")
    return 0


if __name__ == "__main__":
    sys.exit(main())
