#include <zmq.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <random>
#include <cstdlib>
#include "config_reader.h"

int main(int argc, char* argv[]) {
    std::string config_file = "slave_config.conf";
    
    // 检查命令行参数
    if (argc >= 2) {
        config_file = argv[1];
    }
    
    std::cout << "使用配置文件: " << config_file << std::endl;
    
    // 加载配置
    ConfigReader reader(config_file);
    if (!reader.loadConfig()) {
        std::cerr << "加载配置文件失败" << std::endl;
        return 1;
    }
    
    NodeConfig config = reader.getNodeConfig();
    
    if (!reader.validateConfig(config)) {
        std::cerr << "配置验证失败" << std::endl;
        return 1;
    }
    
    if (config.is_master) {
        std::cerr << "错误: 从节点不能使用主节点配置" << std::endl;
        return 1;
    }
    
    std::cout << "从节点配置加载成功:" << std::endl;
    std::cout << "  节点ID: " << config.node_id << std::endl;
    std::cout << "  主节点IP: " << config.master_ip << std::endl;
    std::cout << "  数据端口: " << config.master_data_port << std::endl;
    std::cout << "  命令端口: " << config.master_cmd_port << std::endl;
    
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PUB);
    
    try {
        std::string data_endpoint = "tcp://" + config.master_ip + ":" + 
                                  std::to_string(config.master_data_port);
        socket.connect(data_endpoint);
        std::cout << "节点 " << config.node_id << " 连接到主节点 " << data_endpoint << std::endl;
    } catch (const zmq::error_t& e) {
        std::cerr << "连接失败: " << e.what() << std::endl;
        return 1;
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> char_dist('A', 'Z');
    
    while (true) {
        // 生成随机字符
        char random_char = static_cast<char>(char_dist(gen));
        
        // 发送数据: "节点ID:字符"
        std::string message = std::to_string(config.node_id) + ":" + random_char;
        
        zmq::message_t zmq_msg(message.size());
        memcpy(zmq_msg.data(), message.c_str(), message.size());
        
        try {
            socket.send(zmq_msg, zmq::send_flags::none);
            std::cout << "节点 " << config.node_id << " 发送: " << message << std::endl;
        } catch (const zmq::error_t& e) {
            std::cerr << "发送失败: " << e.what() << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    return 0;
}