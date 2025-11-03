#include <zmq.hpp>
#include <gtk/gtk.h>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <iostream>
#include "config_reader.h"

class GridDisplay {
private:
    GtkWidget *window;
    GtkWidget *grid;
    std::vector<GtkWidget*> labels;
    zmq::context_t context;
    zmq::socket_t data_socket;
    zmq::socket_t cmd_socket;
    std::atomic<bool> running;
    std::thread zmq_thread;
    NodeConfig config;
    
    static const int GRID_SIZE = 3;
    
public:
    GridDisplay(const std::string& config_file = "master_config.conf") 
        : context(1), 
          data_socket(context, ZMQ_SUB), 
          cmd_socket(context, ZMQ_PUB),
          running(false) {
        
        // 加载配置
        ConfigReader reader(config_file);
        if (!reader.loadConfig()) {
            std::cerr << "加载配置文件失败，使用默认配置" << std::endl;
        }
        
        config = reader.getNodeConfig();
        
        if (!reader.validateConfig(config)) {
            std::cerr << "配置验证失败，程序退出" << std::endl;
            exit(1);
        }
        
        std::cout << "主节点配置加载成功:" << std::endl;
        std::cout << "  节点ID: " << config.node_id << std::endl;
        std::cout << "  数据端口: " << config.master_data_port << std::endl;
        std::cout << "  命令端口: " << config.master_cmd_port << std::endl;
        
        // 初始化 GTK
        gtk_init(0, nullptr);
        
        // 创建窗口
        window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_title(GTK_WINDOW(window), 
                           ("ZeroMQ 九宫格监控 - 主节点 " + std::to_string(config.node_id)).c_str());
        gtk_window_set_default_size(GTK_WINDOW(window), 400, 400);
        gtk_container_set_border_width(GTK_CONTAINER(window), 10);
        
        // 创建 3x3 网格
        grid = gtk_grid_new();
        gtk_grid_set_row_spacing(GTK_GRID(grid), 5);
        gtk_grid_set_column_spacing(GTK_GRID(grid), 5);
        gtk_container_add(GTK_CONTAINER(window), grid);
        
        // 创建 9 个标签
        for (int i = 0; i < GRID_SIZE * GRID_SIZE; i++) {
            GtkWidget *frame = gtk_frame_new(nullptr);
            GtkWidget *label = gtk_label_new("等待数据...");
            gtk_label_set_xalign(GTK_LABEL(label), 0.5);
            gtk_label_set_yalign(GTK_LABEL(label), 0.5);
            
            // 设置样式
            gtk_widget_set_size_request(frame, 120, 120);
            gtk_frame_set_shadow_type(GTK_FRAME(frame), GTK_SHADOW_ETCHED_IN);
            
            gtk_container_add(GTK_CONTAINER(frame), label);
            
            int row = i / GRID_SIZE;
            int col = i % GRID_SIZE;
            gtk_grid_attach(GTK_GRID(grid), frame, col, row, 1, 1);
            
            labels.push_back(label);
        }
        
        // 设置 ZeroMQ
        data_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
        data_socket.bind("tcp://*:" + std::to_string(config.master_data_port));
        
        cmd_socket.bind("tcp://*:" + std::to_string(config.master_cmd_port));
        
        // 连接信号
        g_signal_connect(window, "destroy", G_CALLBACK(on_window_destroy), this);
        
        running = true;
        zmq_thread = std::thread(&GridDisplay::zmqReceiver, this);
    }
    
    ~GridDisplay() {
        running = false;
        if (zmq_thread.joinable()) {
            zmq_thread.join();
        }
    }
    
    void show() {
        gtk_widget_show_all(window);
        gtk_main();
    }
    
    NodeConfig getConfig() const {
        return config;
    }
    
private:
    void zmqReceiver() {
        while (running) {
            zmq::message_t message;
            try {
                if (data_socket.recv(message, zmq::recv_flags::dontwait)) {
                    std::string data(static_cast<char*>(message.data()), message.size());
                    processMessage(data);
                }
            } catch (const zmq::error_t& e) {
                if (e.num() != EAGAIN) {
                    std::cerr << "ZeroMQ 错误: " << e.what() << std::endl;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    void processMessage(const std::string& data) {
        // 数据格式: "节点ID:字符"
        size_t pos = data.find(':');
        if (pos != std::string::npos) {
            int node_id = std::stoi(data.substr(0, pos));
            char ch = data.substr(pos + 1)[0];
            
            // 更新 GUI (必须在 GTK 主线程中执行)
            g_idle_add(updateDisplay, new DisplayData{node_id, ch, this});
        }
    }
    
    struct DisplayData {
        int node_id;
        char ch;
        GridDisplay* display;
    };
    
    static gboolean updateDisplay(gpointer user_data) {
        DisplayData* data = static_cast<DisplayData*>(user_data);
        data->display->updateLabel(data->node_id, data->ch);
        delete data;
        return FALSE;
    }
    
    void updateLabel(int node_id, char ch) {
        if (node_id >= 1 && node_id <= 9) {
            int index = node_id - 1;
            
            std::string text = std::string("节点 ") + std::to_string(node_id) + 
                             "\n字符: '" + std::string(1, ch) + "'";
            
            gtk_label_set_text(GTK_LABEL(labels[index]), text.c_str());
            setLabelColor(labels[index], ch);
        }
    }
    
    void setLabelColor(GtkWidget* label, char ch) {
        GtkWidget* parent = gtk_widget_get_parent(label);
        if (GTK_IS_FRAME(parent)) {
            const char* color = "white";
            
            switch (ch) {
                case 'A': case 'a': color = "#FFCCCC"; break;
                case 'B': case 'b': color = "#CCFFCC"; break;
                case 'C': case 'c': color = "#CCCCFF"; break;
                case 'D': case 'd': color = "#FFFFCC"; break;
                case 'E': case 'e': color = "#FFCCFF"; break;
                default: color = "#E0E0E0"; break;
            }
            
            GdkRGBA gdk_color;
            if (gdk_rgba_parse(&gdk_color, color)) {
                gtk_widget_override_background_color(parent, GTK_STATE_FLAG_NORMAL, &gdk_color);
            }
        }
    }
    
    static void on_window_destroy(GtkWidget* widget, gpointer data) {
        GridDisplay* display = static_cast<GridDisplay*>(data);
        display->running = false;
        gtk_main_quit();
    }
};

int main(int argc, char* argv[]) {
    std::string config_file = "master_config.conf";
    
    // 检查命令行参数
    if (argc >= 2) {
        config_file = argv[1];
    }
    
    std::cout << "使用配置文件: " << config_file << std::endl;
    
    GridDisplay display(config_file);
    display.show();
    
    return 0;
}