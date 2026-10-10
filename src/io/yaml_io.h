//
// Created by gaoxiang on 2020/5/13.
//

#ifndef LIGHTNING_YAML_IO_H
#define LIGHTNING_YAML_IO_H

#include <yaml-cpp/yaml.h>
#include <cassert>
#include <string>

namespace lightning {

// 读取yaml配置文件的相关IO
class YAML_IO {
   public:
    explicit YAML_IO(const std::string &path);

    YAML_IO() = default;
    ~YAML_IO() = default;

    inline bool IsOpened() const { return is_opened_; }

    // 保存文件，不指明路径时，覆盖原文件
    bool Save(const std::string &path = "");

    // 获取类型为T的参数值
    template <typename T>
    T GetValue(const std::string &key) const {
        assert(is_opened_);
        return yaml_node_[key].as<T>();
    }

    // 获取在NODE下的key值
    // 读取两层yaml参数
    template <typename T>
    T GetValue(const std::string &node, const std::string &key) const {
        assert(is_opened_);
        T res = yaml_node_[node][key].as<T>();
        return res;
    }

    // 个人修改（2026-10-01）：判断 `node.key` 是否存在且非空。
    //   背景：GetValue 用的是 `as<T>()` 强转，键缺失会抛 YAML 异常，
    //   故【新增的可选配置项】必须先用本函数探测，缺键时走代码默认值，
    //   这样用旧的 yaml 也能跑新二进制（向后兼容）。
    bool HasValue(const std::string &node, const std::string &key) const;

    // 个人修改（2026-10-01）：三层版（与下面的三层 GetValue 配对），例如
    //   `system.publish_map.enable` ⇒ HasValue("system", "publish_map", "enable")
    bool HasValue(const std::string &node_1, const std::string &node_2, const std::string &key) const;

    // 读取三层yaml参数
    template <typename T>
    T GetValue(const std::string &node_1, const std::string &node_2, const std::string &key) const {
        assert(is_opened_);
        T res = yaml_node_[node_1][node_2][key].as<T>();
        return res;
    }

    // 设定类型为T的参数值
    template <typename T>
    void SetValue(const std::string &key, const T &value) {
        yaml_node_[key] = value;
    }

    // 设定NODE下的key值
    template <typename T>
    void SetValue(const std::string &node, const std::string &key, const T &value) {
        yaml_node_[node][key] = value;
    }

   private:
    std::string path_;
    bool is_opened_ = false;
    YAML::Node yaml_node_;
};

}  // namespace lightning

#endif
