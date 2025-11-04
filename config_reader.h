#ifndef CONFIG_READER_H
#define CONFIG_READER_H

#include <string>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

// 节点配置信息
struct NodeConfig {
    bool is_master;                    // 是否是主节点
    int node_id;                       // 节点ID
    std::string master_ip;             // 主节点IP地址
    int master_data_port;              // 主节点数据端口
    int master_cmd_port;               // 主节点命令端口
    int local_data_port;               // 本地数据端口
    int local_cmd_port;                // 本地命令端口
    std::string log_level;             // 日志级别
    int heartbeat_interval;            // 心跳间隔(ms)
    bool open_preview;                 // 是否创建预览窗口

    
    NodeConfig() : is_master(false), node_id(0), 
                  master_data_port(5555), master_cmd_port(5556),
                  local_data_port(6000), local_cmd_port(7000),
                  heartbeat_interval(3000), open_preview(false) {}
};

class ConfigReader {
private:
    std::unordered_map<std::string, std::string> config_map;
    std::string config_file_path;
    
    // 去除字符串两端的空白字符
    std::string trim(const std::string& str) {
        size_t start = str.find_first_not_of(" \t\r\n");
        size_t end = str.find_last_not_of(" \t\r\n");
        if (start == std::string::npos || end == std::string::npos) {
            return "";
        }
        return str.substr(start, end - start + 1);
    }
    
    // 解析配置文件
    bool parseConfigFile() {
        std::ifstream file(config_file_path);
        if (!file.is_open()) {
            std::cerr << "无法打开配置文件: " << config_file_path << std::endl;
            return false;
        }
        
        std::string line;
        int line_num = 0;
        
        while (std::getline(file, line)) {
            line_num++;
            line = trim(line);
            
            // 跳过空行和注释
            if (line.empty() || line[0] == '#' || line[0] == ';') {
                continue;
            }
            
            // 解析键值对
            size_t pos = line.find('=');
            if (pos == std::string::npos) {
                std::cerr << "配置文件格式错误，第 " << line_num << " 行: " << line << std::endl;
                continue;
            }
            
            std::string key = trim(line.substr(0, pos));
            std::string value = trim(line.substr(pos + 1));
            
            if (!key.empty()) {
                config_map[key] = value;
            }
        }
        
        file.close();
        return true;
    }
    
    // 从配置映射中获取值，带默认值
    template<typename T>
    T getValue(const std::string& key, const T& defaultValue) const {
        auto it = config_map.find(key);
        if (it == config_map.end()) {
            return defaultValue;
        }
        
        std::string value = it->second;
        std::istringstream iss(value);
        T result;
        
        if (!(iss >> result)) {
            return defaultValue;
        }
        
        return result;
    }
    
    // 字符串类型的特化版本
    std::string getValue(const std::string& key, const std::string& defaultValue) const {
        auto it = config_map.find(key);
        if (it == config_map.end()) {
            return defaultValue;
        }
        return it->second;
    }
    
    // 布尔类型的特化版本
    bool getValue(const std::string& key, bool defaultValue) const {
        auto it = config_map.find(key);
        if (it == config_map.end()) {
            return defaultValue;
        }
        
        std::string value = it->second;
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);
        
        return (value == "true" || value == "yes" || value == "1" || value == "on");
    }

public:
    ConfigReader(const std::string& file_path = "node_config.conf") 
        : config_file_path(file_path) {}
    
    // 加载配置文件
    bool loadConfig() {
        config_map.clear();
        return parseConfigFile();
    }
    
    // 重新加载配置文件
    bool reloadConfig() {
        return loadConfig();
    }
    
    // 获取节点配置
    NodeConfig getNodeConfig() {
        NodeConfig config;
        
        config.is_master = getValue("node.is_master", false);
        config.node_id = getValue("node.id", 0);
        config.master_ip = getValue("master.ip", std::string("localhost"));
        config.master_data_port = getValue("master.data_port", 5555);
        config.master_cmd_port = getValue("master.cmd_port", 5556);
        config.local_data_port = getValue("local.data_port", 6000);
        config.local_cmd_port = getValue("local.cmd_port", 7000);
        config.log_level = getValue("log.level", std::string("info"));
        config.heartbeat_interval = getValue("heartbeat.interval", 3000);
        config.open_preview = getValue("node.open_preview", false);

        
        return config;
    }
    
    // 获取所有配置项（用于调试）
    void printAllConfig() const {
        std::cout << "=== 配置文件内容 ===" << std::endl;
        for (const auto& pair : config_map) {
            std::cout << pair.first << " = " << pair.second << std::endl;
        }
        std::cout << "====================" << std::endl;
    }
    
    // 验证配置是否有效
    bool validateConfig(const NodeConfig& config) const {
        if (config.is_master) {
            // 主节点验证
            if (config.node_id <= 0) {
                std::cerr << "错误: 主节点ID必须大于0" << std::endl;
                return false;
            }
        } else {
            // 从节点验证
            if (config.node_id <= 0) {
                std::cerr << "错误: 从节点ID必须大于0" << std::endl;
                return false;
            }
            if (config.master_ip.empty()) {
                std::cerr << "错误: 必须指定主节点IP地址" << std::endl;
                return false;
            }
        }
        
        // 端口验证
        if (config.master_data_port <= 0 || config.master_data_port > 65535) {
            std::cerr << "错误: 主节点数据端口无效" << std::endl;
            return false;
        }
        
        if (config.local_data_port <= 0 || config.local_data_port > 65535) {
            std::cerr << "错误: 本地数据端口无效" << std::endl;
            return false;
        }
        
        return true;
    }
    
    // 设置配置文件路径
    void setConfigPath(const std::string& file_path) {
        config_file_path = file_path;
    }
    
    // 获取配置文件路径
    std::string getConfigPath() const {
        return config_file_path;
    }
};

#endif // CONFIG_READER_H