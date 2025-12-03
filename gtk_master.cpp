#include <zmq.hpp>
#include <gtk/gtk.h>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <iostream>
#include <map>
#include "config_reader.h"

class GridDisplay {
private:
    GtkWidget *window;
    GtkWidget *grid;
    std::vector<GtkWidget*> frames;
    std::vector<GtkWidget*> labels;
    zmq::context_t context;
    zmq::socket_t data_socket;
    zmq::socket_t cmd_socket;
    std::atomic<bool> running;
    std::thread zmq_thread;
    NodeConfig config;
    
    // 颜色映射表 - A~K 对应的颜色
    std::map<char, std::string> color_map;
    
    static const int GRID_SIZE = 3;
    
public:
    GridDisplay(const std::string& config_file = "master_config.conf") 
        : context(1), 
          data_socket(context, ZMQ_SUB), 
          cmd_socket(context, ZMQ_PUB),
          running(false) {
        
        // 初始化颜色映射
        initColorMap();
        
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
        gtk_window_set_default_size(GTK_WINDOW(window), 600, 600); // 增大默认窗口大小
        gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
        gtk_container_set_border_width(GTK_CONTAINER(window), 10);
        
        // 允许窗口缩放
        gtk_window_set_resizable(GTK_WINDOW(window), TRUE);
        
        // 创建主容器
        GtkWidget *main_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
        gtk_container_add(GTK_CONTAINER(window), main_box);
        
        // 创建标题标签
        GtkWidget *title_label = gtk_label_new("ZeroMQ 九宫格监控系统");
        gtk_label_set_xalign(GTK_LABEL(title_label), 0.5);
        PangoFontDescription *font_desc = pango_font_description_from_string("Sans Bold 16");
        gtk_widget_override_font(title_label, font_desc);
        pango_font_description_free(font_desc);
        gtk_box_pack_start(GTK_BOX(main_box), title_label, FALSE, FALSE, 0);
        
        // 创建 3x3 网格
        grid = gtk_grid_new();
        gtk_grid_set_row_homogeneous(GTK_GRID(grid), TRUE);    // 行均匀分布
        gtk_grid_set_column_homogeneous(GTK_GRID(grid), TRUE); // 列均匀分布
        gtk_grid_set_row_spacing(GTK_GRID(grid), 5);
        gtk_grid_set_column_spacing(GTK_GRID(grid), 5);
        
        // 设置网格在容器中可扩展
        gtk_box_pack_start(GTK_BOX(main_box), grid, TRUE, TRUE, 0);
        
        // 创建 9 个格子
        for (int i = 0; i < GRID_SIZE * GRID_SIZE; i++) {
            // 创建框架
            GtkWidget *frame = gtk_frame_new(nullptr);
            gtk_frame_set_shadow_type(GTK_FRAME(frame), GTK_SHADOW_ETCHED_IN);
            gtk_widget_set_hexpand(frame, TRUE);  // 水平扩展
            gtk_widget_set_vexpand(frame, TRUE);  // 垂直扩展
            
            // 创建事件盒子用于设置背景色
            GtkWidget *event_box = gtk_event_box_new();
            gtk_container_add(GTK_CONTAINER(frame), event_box);
            
            // 创建标签
            GtkWidget *label = gtk_label_new("等待数据...");
            gtk_label_set_xalign(GTK_LABEL(label), 0.5);
            gtk_label_set_yalign(GTK_LABEL(label), 0.5);
            gtk_label_set_line_wrap(GTK_LABEL(label), TRUE);  // 允许文本换行
            gtk_label_set_justify(GTK_LABEL(label), GTK_JUSTIFY_CENTER);
            
            // 设置标签字体
            PangoFontDescription *label_font = pango_font_description_from_string("Sans 12");
            gtk_widget_override_font(label, label_font);
            pango_font_description_free(label_font);
            
            gtk_container_add(GTK_CONTAINER(event_box), label);
            
            // 设置初始背景色
            GdkRGBA default_color;
            gdk_rgba_parse(&default_color, "#F0F0F0");
            gtk_widget_override_background_color(event_box, GTK_STATE_FLAG_NORMAL, &default_color);
            
            int row = i / GRID_SIZE;
            int col = i % GRID_SIZE;
            gtk_grid_attach(GTK_GRID(grid), frame, col, row, 1, 1);
            
            frames.push_back(frame);
            labels.push_back(label);
        }
        
        // 创建状态栏
        GtkWidget *status_bar = gtk_statusbar_new();
        gtk_box_pack_start(GTK_BOX(main_box), status_bar, FALSE, FALSE, 0);
        gtk_statusbar_push(GTK_STATUSBAR(status_bar), 0, "就绪 - 等待节点连接...");
        
        // 设置 ZeroMQ
        data_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
        data_socket.bind("tcp://*:" + std::to_string(config.master_data_port));
        
        cmd_socket.bind("tcp://*:" + std::to_string(config.master_cmd_port));
        
        // 连接信号
        g_signal_connect(window, "destroy", G_CALLBACK(on_window_destroy), this);
        g_signal_connect(window, "configure-event", G_CALLBACK(on_window_resize), this);
        
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
    // 初始化颜色映射表
    void initColorMap() {
        color_map = {
            {'A', "#FF6B6B"},  // 红色
            {'B', "#4ECDC4"},  // 青绿色
            {'C', "#45B7D1"},  // 蓝色
            {'D', "#96CEB4"},  // 绿色
            {'E', "#FFEAA7"},  // 黄色
            {'F', "#DDA0DD"},  // 紫色
            {'G', "#FFA07A"},  // 橙色
            {'H', "#98D8C8"},  // 薄荷绿
            {'I', "#F7DC6F"},  // 金色
            {'J', "#BB8FCE"},  // 淡紫色
            {'K', "#85C1E9"},  // 天蓝色
            // {'0', "#FF6B6B"},  // 红色
            // {'1', "#4ECDC4"},  // 青绿色
            // {'2', "#45B7D1"},  // 蓝色
            // {'3', "#96CEB4"},  // 绿色
            // {'4', "#FFEAA7"},  // 黄色
            // {'5', "#DDA0DD"},  // 紫色
            // {'6', "#FFA07A"},  // 橙色
            // {'7', "#98D8C8"},  // 薄荷绿
            // {'8', "#F7DC6F"},  // 金色
            // {'9', "#BB8FCE"}   // 淡紫色
                {'0', "#FF6B6B"},  // 红色 (红)
            {'1', "#96CEB4"},  // 绿色 (绿)
            {'2', "#45B7D1"},  // 蓝色 (蓝)
            {'3', "#FF6B6B"},  // 红色 (红)
            {'4', "#96CEB4"},  // 绿色 (绿)
            {'5', "#45B7D1"},  // 蓝色 (蓝)
            {'6', "#FF6B6B"},  // 红色 (红)
            {'7', "#96CEB4"},  // 绿色 (绿)
            {'8', "#45B7D1"},  // 蓝色 (蓝)
            {'9', "#FF6B6B"}   // 红色 (红)
        };
    }
    
    // 根据字符获取颜色
    std::string getColorForChar(char ch) {
        char upper_ch = std::toupper(ch);
        auto it = color_map.find(upper_ch);
        if (it != color_map.end()) {
            return it->second;
        }
        return "#E0E0E0";  // 默认灰色
    }
    
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
            
            // 更新标签文本
            std::string text = std::string("节点 ") + std::to_string(node_id) + 
                             "\n字符: '" + std::string(1, ch) + "'" +
                             "\n时间: " + getCurrentTime();
            
            gtk_label_set_text(GTK_LABEL(labels[index]), text.c_str());
            
            // 更新背景颜色
            updateLabelColor(index, ch);
            
            std::cout << "更新节点 " << node_id << " 显示: 字符 '" << ch << "'" << std::endl;
        }
    }
    
    void updateLabelColor(int index, char ch) {
        if (index >= 0 && index < frames.size()) {
            GtkWidget *frame = frames[index];
            GtkWidget *event_box = gtk_bin_get_child(GTK_BIN(frame));
            
            if (event_box && GTK_IS_EVENT_BOX(event_box)) {
                std::string color_str = getColorForChar(ch);
                GdkRGBA color;
                if (gdk_rgba_parse(&color, color_str.c_str())) {
                    gtk_widget_override_background_color(event_box, GTK_STATE_FLAG_NORMAL, &color);
                    
                    // 根据背景色亮度调整文字颜色
                    GdkRGBA text_color;
                    if (isLightColor(color)) {
                        gdk_rgba_parse(&text_color, "#000000");  // 浅背景用黑色文字
                    } else {
                        gdk_rgba_parse(&text_color, "#FFFFFF");  // 深背景用白色文字
                    }
                    
                    GtkWidget *label = gtk_bin_get_child(GTK_BIN(event_box));
                    if (label && GTK_IS_LABEL(label)) {
                        gtk_widget_override_color(label, GTK_STATE_FLAG_NORMAL, &text_color);
                    }
                }
            }
        }
    }
    
    // 判断颜色是否为浅色
    bool isLightColor(const GdkRGBA& color) {
        // 计算亮度 (使用相对亮度公式)
        double brightness = (0.299 * color.red + 0.587 * color.green + 0.114 * color.blue) * 255;
        return brightness > 128;
    }
    
    // 获取当前时间字符串
    std::string getCurrentTime() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");
        ss << "." << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }
    
    static void on_window_destroy(GtkWidget* widget, gpointer data) {
        GridDisplay* display = static_cast<GridDisplay*>(data);
        display->running = false;
        gtk_main_quit();
    }
    
    static gboolean on_window_resize(GtkWidget* widget, GdkEventConfigure* event, gpointer data) {
        GridDisplay* display = static_cast<GridDisplay*>(data);
        
        // 获取窗口新尺寸
        int width = event->width;
        int height = event->height;
        
        // 可以根据窗口大小调整字体大小等
        // 这里只是示例，实际可以根据需要实现更复杂的自适应逻辑
        std::cout << "窗口大小改变: " << width << " x " << height << std::endl;
        
        return FALSE;
    }
};

int main(int argc, char* argv[]) {
    std::string config_file = "master_config.conf";
    
    // 检查命令行参数
    if (argc >= 2) {
        config_file = argv[1];
    }
    
    std::cout << "使用配置文件: " << config_file << std::endl;
    std::cout << "支持的字符颜色映射:" << std::endl;
    std::cout << "  A: 红色    B: 青绿色  C: 蓝色" << std::endl;
    std::cout << "  D: 绿色    E: 黄色    F: 紫色" << std::endl;
    std::cout << "  G: 橙色    H: 薄荷绿  I: 金色" << std::endl;
    std::cout << "  J: 淡紫色  K: 天蓝色  其他: 灰色" << std::endl;
    
    GridDisplay display(config_file);
    display.show();
    
    return 0;
}