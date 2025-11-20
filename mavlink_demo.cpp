/**
 * PC 端 MAVLink 双向通信 Demo (使用标准消息)
 * 发送命令控制 Arduino LED 和 RGB LED，接收传感器数据
 */

 #include <iostream>
 #include <thread>
 #include <atomic>
 #include <vector>
 #include <string>
 #include <iomanip>
 #include <chrono>
 #include <sstream>
 
 // MAVLink 头文件
 #include "common/mavlink.h"
 
 // 串口通信头文件 (Linux)
 #include <fcntl.h>
 #include <termios.h>
 #include <unistd.h>
 
 class MavlinkCommunicator {
 private:
     // 串口配置
     std::string port_;
     int baudrate_;
     int serial_fd_;
     
     // 系统配置
     uint8_t system_id_;
     uint8_t component_id_;
     uint8_t target_system_;
     uint8_t target_component_;
     
     // 控制标志
     std::atomic<bool> running_;
     std::thread receive_thread_;
     std::thread send_thread_;
     
     // 统计信息
     uint32_t packets_received_;
     uint32_t packets_sent_;
     uint32_t heartbeat_count_;
     
     // 最新数据
     float last_voltage_;
     float last_temperature_;
     uint16_t last_sensor_raw_;
     uint32_t last_heartbeat_time_;
     bool connection_ok_;
     float battery_voltage_;
     int battery_percentage_;
     
     // RGB 状态
     uint8_t rgb_red_;
     uint8_t rgb_green_;
     uint8_t rgb_blue_;
 
 public:
     MavlinkCommunicator(const std::string& port = "/dev/ttyS1", 
                        int baudrate = 57600,
                        uint8_t sys_id = 255, 
                        uint8_t comp_id = 0,
                        uint8_t target_sys = 1,
                        uint8_t target_comp = 1)
         : port_(port), baudrate_(baudrate), 
           system_id_(sys_id), component_id_(comp_id),
           target_system_(target_sys), target_component_(target_comp),
           running_(false), serial_fd_(-1),
           packets_received_(0), packets_sent_(0), heartbeat_count_(0),
           last_voltage_(0.0f), last_temperature_(0.0f), last_sensor_raw_(0),
           last_heartbeat_time_(0), connection_ok_(false),
           battery_voltage_(0.0f), battery_percentage_(0),
           rgb_red_(0), rgb_green_(0), rgb_blue_(0) {
     }
     
     ~MavlinkCommunicator() {
         disconnect();
     }
     
     // 连接到串口
     bool connect() {
         serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
         if (serial_fd_ < 0) {
             std::cerr << "Error opening serial port: " << port_ << std::endl;
             return false;
         }
         
         // 配置串口
         struct termios tty;
         if (tcgetattr(serial_fd_, &tty) != 0) {
             std::cerr << "Error getting terminal attributes" << std::endl;
             return false;
         }
         
         cfsetospeed(&tty, B57600);
         cfsetispeed(&tty, B57600);
         
         tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8位数据
         tty.c_iflag &= ~IGNBRK; // 禁用中断处理
         tty.c_lflag = 0; // 无信号字符，无回显
         tty.c_oflag = 0; // 无remapping，无延迟
         tty.c_cc[VMIN]  = 0; // 读取不需要字符
         tty.c_cc[VTIME] = 5; // 0.5秒读取超时
         
         tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控
         tty.c_cflag |= (CLOCAL | CREAD); // 忽略调制解调器控制线，启用接收
         tty.c_cflag &= ~(PARENB | PARODD); // 无奇偶校验
         tty.c_cflag &= ~CSTOPB; // 1位停止位
         tty.c_cflag &= ~CRTSCTS; // 禁用硬件流控
         
         if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
             std::cerr << "Error setting terminal attributes" << std::endl;
             return false;
         }
         
         std::cout << "Connected to " << port_ << " at " << baudrate_ << " baud" << std::endl;
         return true;
     }
     
     // 断开连接
     void disconnect() {
         running_ = false;
         
         if (receive_thread_.joinable()) {
             receive_thread_.join();
         }
         if (send_thread_.joinable()) {
             send_thread_.join();
         }
         
         if (serial_fd_ >= 0) {
             close(serial_fd_);
             serial_fd_ = -1;
         }
         
         std::cout << "Disconnected" << std::endl;
     }
     
     // 开始通信
     bool start() {
         if (!connect()) {
             return false;
         }
         
         running_ = true;
         
         // 启动接收线程
         receive_thread_ = std::thread(&MavlinkCommunicator::receiveLoop, this);
         
         // 启动发送线程
         send_thread_ = std::thread(&MavlinkCommunicator::sendLoop, this);
         
         std::cout << "MAVLink communication started" << std::endl;
         return true;
     }
     
     // 接收循环
     void receiveLoop() {
         mavlink_status_t status;
         mavlink_message_t msg;
         
         uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
         
         while (running_) {
             // 读取串口数据
             int bytes_read = read(serial_fd_, buffer, sizeof(buffer));
             
             if (bytes_read > 0) {
                 for (int i = 0; i < bytes_read; i++) {
                     // 解析 MAVLink 消息
                     if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                         handleMavlinkMessage(&msg);
                         packets_received_++;
                     }
                 }
             } else if (bytes_read < 0) {
                 std::cerr << "Error reading from serial port" << std::endl;
                 std::this_thread::sleep_for(std::chrono::milliseconds(100));
             }
         }
     }
     
     // 发送循环
     void sendLoop() {
         auto last_heartbeat = std::chrono::steady_clock::now();
         auto last_status = std::chrono::steady_clock::now();
         
         while (running_) {
             auto now = std::chrono::steady_clock::now();
             
             // 发送心跳 (1Hz)
             if (now - last_heartbeat >= std::chrono::seconds(1)) {
                 sendHeartbeat();
                 last_heartbeat = now;
             }
             
             // 检查连接状态 (每5秒)
             if (now - last_status >= std::chrono::seconds(5)) {
                 checkConnection();
                 displayStatus();
                 last_status = now;
             }
             
             std::this_thread::sleep_for(std::chrono::milliseconds(100));
         }
     }
     
     // 处理接收到的 MAVLink 消息
     void handleMavlinkMessage(mavlink_message_t* msg) {
         switch (msg->msgid) {
             case MAVLINK_MSG_ID_HEARTBEAT:
                 {
                     mavlink_heartbeat_t heartbeat;
                     mavlink_msg_heartbeat_decode(msg, &heartbeat);
                     last_heartbeat_time_ = getCurrentTimeMs();
                     heartbeat_count_++;
                     connection_ok_ = true;
                     
                     std::cout << "Heartbeat from system " << (int)msg->sysid 
                               << ", type: " << (int)heartbeat.type 
                               << ", autopilot: " << (int)heartbeat.autopilot << std::endl;
                 }
                 break;
                 
             case MAVLINK_MSG_ID_SCALED_PRESSURE:
                 {
                     mavlink_scaled_pressure_t sensor;
                     mavlink_msg_scaled_pressure_decode(msg, &sensor);
                     
                     // 解析复用字段
                     last_voltage_ = sensor.press_abs;        // 电压值
                     last_temperature_ = sensor.press_diff;   // 温度值
                     last_sensor_raw_ = sensor.temperature;   // 原始传感器值
                     
                     std::cout << "\n=== Sensor Data ===" << std::endl;
                     std::cout << "Voltage: " << std::fixed << std::setprecision(2) 
                               << sensor.press_abs << " V" << std::endl;
                     std::cout << "Temperature: " << sensor.press_diff << " C" << std::endl;
                     std::cout << "Raw Sensor: " << sensor.temperature << std::endl;
                     std::cout << "Timestamp: " << sensor.time_boot_ms << " ms" << std::endl;
                 }
                 break;
                 
             case MAVLINK_MSG_ID_SYS_STATUS:
                 {
                     mavlink_sys_status_t sys_status;
                     mavlink_msg_sys_status_decode(msg, &sys_status);
                     
                     battery_voltage_ = sys_status.voltage_battery / 1000.0f; // 转换为伏特
                     battery_percentage_ = sys_status.battery_remaining;
                     
                     std::cout << "=== System Status ===" << std::endl;
                     std::cout << "Battery: " << battery_voltage_ << " V, " 
                               << battery_percentage_ << "%" << std::endl;
                 }
                 break;
                 
             case MAVLINK_MSG_ID_STATUSTEXT:
                 {
                     mavlink_statustext_t status_text;
                     mavlink_msg_statustext_decode(msg, &status_text);
                     std::cout << "Arduino: " << status_text.text << std::endl;
                 }
                 break;
                                 
             default:
                 std::cout << "Received message ID: " << msg->msgid << std::endl;
                 break;
         }
     }
     
     // 发送心跳
     void sendHeartbeat() {
         mavlink_message_t msg;
         
         mavlink_msg_heartbeat_pack(
             system_id_, component_id_,
             &msg,
             MAV_TYPE_GCS,           // 类型：地面站
             MAV_AUTOPILOT_INVALID,  // 自动驾驶仪：无
             MAV_MODE_MANUAL_ARMED,  // 模式：手动已解锁
             0,                      // 自定义模式
             MAV_STATE_ACTIVE        // 状态：活跃
         );
         
         sendMavlinkMessage(&msg);
         packets_sent_++;
     }
     
     // 发送 LED 控制命令
     void sendLEDCommand(bool led_on) {
         mavlink_message_t msg;
         
         mavlink_msg_command_long_pack(
             system_id_, component_id_,
             &msg,
             target_system_,     // 目标系统
             target_component_,  // 目标组件
             MAV_CMD_USER_1,     // 用户自定义命令1：控制 LED
             0,                  // 确认
             led_on ? 1.0f : 0.0f, // 参数1：LED状态
             0, 0, 0, 0, 0, 0   // 其他参数
         );
         
         sendMavlinkMessage(&msg);
         packets_sent_++;
         
         std::cout << "Sent LED command: " << (led_on ? "ON" : "OFF") << std::endl;
     }
     
     // 发送读取传感器命令
     void sendReadSensorCommand() {
         mavlink_message_t msg;
         
         mavlink_msg_command_long_pack(
             system_id_, component_id_,
             &msg,
             target_system_,     // 目标系统
             target_component_,  // 目标组件
             MAV_CMD_USER_2,     // 用户自定义命令2：读取传感器
             0,                  // 确认
             0, 0, 0, 0, 0, 0, 0 // 参数
         );
         
         sendMavlinkMessage(&msg);
         packets_sent_++;
         
         std::cout << "Sent read sensor command" << std::endl;
     }
     
     // 发送 RGB LED 颜色命令
     void sendRGBCommand(uint8_t red, uint8_t green, uint8_t blue) {
        mavlink_message_t msg;
        
        // 将 RGB 值 (0-255) 映射到 RC 通道值 (1000-2000)
        uint16_t rc_red = red;
        uint16_t rc_green = green;
        uint16_t rc_blue = blue;
        
        mavlink_msg_rc_channels_override_pack(
            system_id_, component_id_,
            &msg,
            target_system_,     // 目标系统
            target_component_,  // 目标组件
            rc_red,             // 通道1: 红色
            rc_green,           // 通道2: 绿色
            rc_blue,            // 通道3: 蓝色
            UINT16_MAX,         // 通道4: 未使用
            UINT16_MAX,         // 通道5: 未使用
            UINT16_MAX,         // 通道6: 未使用
            UINT16_MAX,         // 通道7: 未使用
            UINT16_MAX,         // 通道8: 未使用
            UINT16_MAX,         // 通道1: 未使用
            UINT16_MAX,         // 通道2: 未使用
            UINT16_MAX,         // 通道3: 未使用
            UINT16_MAX,         // 通道4: 未使用
            UINT16_MAX,         // 通道5: 未使用
            UINT16_MAX,         // 通道6: 未使用
            UINT16_MAX,         // 通道7: 未使用
            UINT16_MAX,         // 通道8: 未使用
            UINT16_MAX,         // 通道7: 未使用
            UINT16_MAX          // 通道8: 未使用
        );
        
        sendMavlinkMessage(&msg);
        packets_sent_++;
        
        std::cout << "Sent RGB via RC_OVERRIDE: R=" << (int)red 
                  << " G=" << (int)green << " B=" << (int)blue << std::endl;
    }
     
     // 发送 RGB 特效命令
     void sendRGBEffectCommand(uint8_t effect) {
         mavlink_message_t msg;
         
         mavlink_msg_command_long_pack(
             system_id_, component_id_,
             &msg,
             target_system_,     // 目标系统
             target_component_,  // 目标组件
             MAV_CMD_USER_4,     // 用户自定义命令4：RGB 特效
             0,                  // 确认
             (float)effect,      // 参数1：特效类型
             0, 0, 0, 0, 0, 0   // 其他参数
         );
         
         sendMavlinkMessage(&msg);
         packets_sent_++;
         
         std::string effect_name;
         switch (effect) {
             case 0: effect_name = "Off"; break;
             case 1: effect_name = "Red"; break;
             case 2: effect_name = "Green"; break;
             case 3: effect_name = "Blue"; break;
             case 4: effect_name = "Yellow"; break;
             case 5: effect_name = "Purple"; break;
             case 6: effect_name = "Cyan"; break;
             case 7: effect_name = "White"; break;
             case 8: effect_name = "Breathing Red"; break;
             default: effect_name = "Unknown"; break;
         }
         
         std::cout << "Sent RGB effect: " << effect_name << " (" << (int)effect << ")" << std::endl;
     }
     
     // 请求系统状态消息
     void requestSystemStatus() {
         mavlink_message_t msg;
         
         mavlink_msg_command_long_pack(
             system_id_, component_id_,
             &msg,
             target_system_,     // 目标系统
             target_component_,  // 目标组件
             MAV_CMD_REQUEST_MESSAGE, // 请求消息命令
             0,                  // 确认
             MAVLINK_MSG_ID_SYS_STATUS, // 参数1：请求的消息ID
             0, 0, 0, 0, 0, 0   // 其他参数
         );
         
         sendMavlinkMessage(&msg);
         packets_sent_++;
         
         std::cout << "Requested system status" << std::endl;
     }
     
     // 发送 MAVLink 消息
     void sendMavlinkMessage(mavlink_message_t* msg) {
         uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
         uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);
         
         if (write(serial_fd_, buffer, len) != len) {
             std::cerr << "Error writing to serial port" << std::endl;
         }
     }
     
     // 检查连接状态
     void checkConnection() {
         uint32_t current_time = getCurrentTimeMs();
         if (current_time - last_heartbeat_time_ > 5000) { // 5秒无心跳认为断开
             connection_ok_ = false;
         }
     }
     
     // 显示状态信息
     void displayStatus() {
         std::cout << "\n=== MAVLink Status ===" << std::endl;
         std::cout << "Connection: " << (connection_ok_ ? "OK" : "LOST") << std::endl;
         std::cout << "Packets Received: " << packets_received_ << std::endl;
         std::cout << "Packets Sent: " << packets_sent_ << std::endl;
         std::cout << "Heartbeats: " << heartbeat_count_ << std::endl;
         std::cout << "Latest Voltage: " << last_voltage_ << " V" << std::endl;
         std::cout << "Latest Temperature: " << last_temperature_ << " C" << std::endl;
         std::cout << "Battery: " << battery_voltage_ << " V, " << battery_percentage_ << "%" << std::endl;
         std::cout << "RGB Color: R=" << (int)rgb_red_ << " G=" << (int)rgb_green_ << " B=" << (int)rgb_blue_ << std::endl;
     }
     
     // 获取当前时间（毫秒）
     uint32_t getCurrentTimeMs() {
         auto now = std::chrono::steady_clock::now();
         return std::chrono::duration_cast<std::chrono::milliseconds>(
             now.time_since_epoch()).count();
     }
     
     // 解析 RGB 输入
     bool parseRGBInput(const std::string& input, uint8_t& red, uint8_t& green, uint8_t& blue) {
         std::stringstream ss(input);
         std::string token;
         std::vector<int> values;
         
         while (std::getline(ss, token, ',')) {
             try {
                 values.push_back(std::stoi(token));
             } catch (const std::exception& e) {
                 return false;
             }
         }
         
         if (values.size() != 3) {
             return false;
         }
         
         red = (uint8_t)constrain(values[0], 0, 255);
         green = (uint8_t)constrain(values[1], 0, 255);
         blue = (uint8_t)constrain(values[2], 0, 255);
         
         return true;
     }
     
     // 限制数值范围
     int constrain(int value, int min_val, int max_val) {
         if (value < min_val) return min_val;
         if (value > max_val) return max_val;
         return value;
     }
     
     // 用户交互界面
     void runUserInterface() {
         std::cout << "\n=== MAVLink Arduino Controller with RGB ===" << std::endl;
         std::cout << "1. Turn LED ON" << std::endl;
         std::cout << "2. Turn LED OFF" << std::endl;
         std::cout << "3. Read Sensor" << std::endl;
         std::cout << "4. Request System Status" << std::endl;
         std::cout << "5. Set RGB Color (format: R,G,B)" << std::endl;
         std::cout << "6. RGB Effects" << std::endl;
         std::cout << "7. Show Status" << std::endl;
         std::cout << "q. Quit" << std::endl;
         
         char choice;
         std::string input;
         
         while (running_) {
             std::cout << "\nEnter choice: ";
             std::cin >> choice;
             
             switch (choice) {
                 case '1':
                     sendLEDCommand(true);
                     break;
                 case '2':
                     sendLEDCommand(false);
                     break;
                 case '3':
                     sendReadSensorCommand();
                     break;
                 case '4':
                     requestSystemStatus();
                     break;
                 case '5':
                     std::cout << "Enter RGB values (0-255) as R,G,B: ";
                     std::cin >> input;
                     {
                         uint8_t red, green, blue;
                         if (parseRGBInput(input, red, green, blue)) {
                             sendRGBCommand(red, green, blue);
                         } else {
                             std::cout << "Invalid input! Use format: 255,128,0" << std::endl;
                         }
                     }
                     break;
                 case '6':
                     std::cout << "RGB Effects:" << std::endl;
                     std::cout << "  0: Off" << std::endl;
                     std::cout << "  1: Red" << std::endl;
                     std::cout << "  2: Green" << std::endl;
                     std::cout << "  3: Blue" << std::endl;
                     std::cout << "  4: Yellow" << std::endl;
                     std::cout << "  5: Purple" << std::endl;
                     std::cout << "  6: Cyan" << std::endl;
                     std::cout << "  7: White" << std::endl;
                     std::cout << "  8: Breathing Red" << std::endl;
                     std::cout << "Enter effect number: ";
                     {
                         int effect;
                         std::cin >> effect;
                         if (effect >= 0 && effect <= 8) {
                             sendRGBEffectCommand((uint8_t)effect);
                         } else {
                             std::cout << "Invalid effect number!" << std::endl;
                         }
                     }
                     break;
                 case '7':
                     displayStatus();
                     break;
                 case 'q':
                     running_ = false;
                     break;
                 default:
                     std::cout << "Invalid choice!" << std::endl;
                     break;
             }
             
             std::this_thread::sleep_for(std::chrono::milliseconds(100));
         }
     }
 };
 
 int main(int argc, char** argv) {
     std::string port = "/dev/ttyS1";
     if (argc > 1) {
         port = argv[1];
     }
     
     MavlinkCommunicator comm(port, 57600);
     
     if (!comm.start()) {
         std::cerr << "Failed to start MAVLink communication" << std::endl;
         return 1;
     }
     
     // 运行用户界面
     comm.runUserInterface();
     
     std::cout << "Shutting down..." << std::endl;
     return 0;
 }