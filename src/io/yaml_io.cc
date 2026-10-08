//
// Created by gaoxiang on 2020/5/13.
//

#include "io/yaml_io.h"

#include <glog/logging.h>
#include <fstream>

namespace lightning {
YAML_IO::YAML_IO(const std::string &path) {
    path_ = path;
    yaml_node_ = YAML::LoadFile(path_);
    if (yaml_node_.IsNull()) {
        LOG(ERROR) << "Failed to open yaml: " << path_;
    }

    is_opened_ = true;
}

namespace {

/// 个人修改（2026-10-01）：安全地取子节点（可选键探测的基石）。
///   实测 yaml-cpp（0.7/0.8）：对【无效/未定义】节点调用 `IsMap()`、`as<T>()` 会抛
///   `YAML::InvalidNode`（报 "invalid node; first invalid key: xxx"）；只有
///   `IsDefined()` / `IsNull()` 在任何节点上都是安全的；而对【合法 map】取不存在的键
///   只是返回"无效节点"、并不抛异常。
///   因此探测顺序必须是：IsDefined/IsNull → IsMap → operator[]，任何一步不满足就返回 false。
bool GetChildNode(const YAML::Node &parent, const std::string &key, YAML::Node &child) {
    if (!parent.IsDefined() || parent.IsNull() || !parent.IsMap()) {
        return false;
    }

    const YAML::Node c = parent[key];  // 合法 map + 键不存在 ⇒ 返回无效节点（不抛）
    if (!c.IsDefined() || c.IsNull()) {
        return false;
    }

    child = c;
    return true;
}

}  // namespace

bool YAML_IO::HasValue(const std::string &node, const std::string &key) const {
    YAML::Node sub;
    if (!GetChildNode(yaml_node_, node, sub)) {
        return false;
    }

    YAML::Node value;
    return GetChildNode(sub, key, value);
}

bool YAML_IO::HasValue(const std::string &node_1, const std::string &node_2, const std::string &key) const {
    YAML::Node level1;
    if (!GetChildNode(yaml_node_, node_1, level1)) {
        return false;
    }

    YAML::Node level2;
    if (!GetChildNode(level1, node_2, level2)) {
        return false;
    }

    YAML::Node value;
    return GetChildNode(level2, key, value);
}

bool YAML_IO::Save(const std::string &path) {
    if (path.empty()) {
        std::ofstream fout(path_);
        fout << yaml_node_;
    } else {
        std::ofstream fout(path);
        fout << yaml_node_;
    }
    return true;
}

}  // namespace lightning
