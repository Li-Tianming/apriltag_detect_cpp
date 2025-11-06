# 编译器设置
CXX := g++
CXXFLAGS := -std=c++11 -Wall -Wextra

# OpenCV 配置
OPENCV_FLAGS := $(shell pkg-config --cflags opencv4)
OPENCV_LIBS := $(shell pkg-config --libs opencv4) -L ./lib/aarch64/static -ltstc_usbcam -ludev -pthread

# AprilTag paths (adjust if installed in non-standard locations)
APRILTAG_INC := /usr/local/include/apriltag
APRILTAG_LIB := /usr/local/lib
APRILTAG_LIBS := -L$(APRILTAG_LIB) -lapriltag

# ZeroMQ 和 GTK 配置
ZMQ_LIBS := -lzmq -lpthread
GTK_PKG := gtk+-3.0
GTK_FLAGS := $(shell pkg-config --cflags $(GTK_PKG))
GTK_LIBS := $(shell pkg-config --libs $(GTK_PKG))

# 目标设置
DIR_INC := ./include/
TARGET := opencv_program
ZMQ_MASTER_TARGET := gtk_master
ZMQ_SLAVE_TARGET := simple_slave

# 源文件
MAIN_SRC := main.cpp
ZMQ_MASTER_SRC := gtk_master.cpp
ZMQ_SLAVE_SRC := simple_slave.cpp
CONFIG_READER_H := config_reader.h

OBJ := $(MAIN_SRC:.cpp=.o)
ZMQ_MASTER_OBJ := $(ZMQ_MASTER_SRC:.cpp=.o)
ZMQ_SLAVE_OBJ := $(ZMQ_SLAVE_SRC:.cpp=.o)

# 构建模式（默认debug）
BUILD ?= debug

# 模式特定配置
ifeq ($(BUILD),release)
        CXXFLAGS += -O3 -DNDEBUG
else
        CXXFLAGS += -g -O0
endif

# 默认目标 - 构建所有目标
all: check_deps $(TARGET) $(ZMQ_MASTER_TARGET) $(ZMQ_SLAVE_TARGET)

# 检查依赖
check_deps:
	@echo "检查依赖库..."
	@pkg-config --exists $(GTK_PKG) || (echo "警告: GTK+3.0 开发包未安装，将无法编译 ZeroMQ 监控程序" && echo "运行: sudo apt-get install libgtk-3-dev")
	@ldconfig -p | grep libzmq > /dev/null || (echo "警告: libzmq 可能未安装" && echo "运行: sudo apt-get install libzmq3-dev")
	@pkg-config --exists opencv4 || (echo "警告: OpenCV4 开发包未安装" && echo "运行: sudo apt-get install libopencv-dev")

# 主 OpenCV 程序
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(OPENCV_LIBS) $(APRILTAG_LIBS) $(ZMQ_LIBS)

# ZeroMQ 主节点程序
$(ZMQ_MASTER_TARGET): $(ZMQ_MASTER_OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(ZMQ_LIBS) $(GTK_LIBS)

# ZeroMQ 从节点程序
$(ZMQ_SLAVE_TARGET): $(ZMQ_SLAVE_OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(ZMQ_LIBS)

# 编译规则 - OpenCV 主程序
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) -I$(APRILTAG_INC) -c $< -o $@

# 编译规则 - ZeroMQ 主节点
$(ZMQ_MASTER_OBJ): $(ZMQ_MASTER_SRC) $(CONFIG_READER_H)
	$(CXX) $(CXXFLAGS) $(GTK_FLAGS) -c $< -o $@

# 编译规则 - ZeroMQ 从节点
$(ZMQ_SLAVE_OBJ): $(ZMQ_SLAVE_SRC) $(CONFIG_READER_H)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 清理
clean:
	rm -f $(OBJ) $(ZMQ_MASTER_OBJ) $(ZMQ_SLAVE_OBJ) $(TARGET) $(ZMQ_MASTER_TARGET) $(ZMQ_SLAVE_TARGET) *.conf

# 安装依赖 (Ubuntu/Debian)
install_deps:
	@echo "安装必要的依赖库..."
	sudo apt-get update
	sudo apt-get install -y libgtk-3-dev libzmq3-dev pkg-config libopencv-dev
	@echo "注意: AprilTag 需要手动安装"

# 创建示例配置文件
create_configs:
	@echo "创建示例配置文件..."
	@echo "# 主节点配置文件\nnode.is_master = true\nnode.id = 1\nmaster.data_port = 5555\nmaster.cmd_port = 5556\nlocal.data_port = 6000\nlocal.cmd_port = 7000\nheartbeat.interval = 3000\nlog.level = info" > master_config.conf
	@echo "# 从节点配置文件\nnode.is_master = false\nnode.id = 2\nmaster.ip = localhost\nmaster.data_port = 5555\nmaster.cmd_port = 5556\nlocal.data_port = 6002\nlocal.cmd_port = 7002\nheartbeat.interval = 3000\nlog.level = info\nnode.open_preview = false\nnode.relative_distance  = false" > slave_config.conf
	@echo "示例配置文件创建完成: master_config.conf, slave_config.conf"

# 运行 OpenCV 主程序
run_opencv: $(TARGET)
	./$(TARGET)

# 运行 ZeroMQ 主节点
run_master: $(ZMQ_MASTER_TARGET) create_configs
	./$(ZMQ_MASTER_TARGET) master_config.conf

# 运行 ZeroMQ 从节点
run_slave: $(ZMQ_SLAVE_TARGET) create_configs
	@if [ -z "$(id)" ]; then \
		echo "用法: make run_slave id=节点ID"; \
		echo "示例: make run_slave id=2"; \
	else \
		sed -i "s/node.id = .*/node.id = $(id)/" slave_config.conf; \
		./$(ZMQ_SLAVE_TARGET) slave_config.conf; \
	fi

# 运行所有从节点 (1-9)
run_all_slaves: $(ZMQ_SLAVE_TARGET) create_configs
	@echo "启动所有从节点 (1-9)..."
	@for i in 1 2 3 4 5 6 7 8 9; do \
		sed -i "s/node.id = .*/node.id = $$i/" slave_config.conf; \
		gnome-terminal --title="节点 $$i" -- ./$(ZMQ_SLAVE_TARGET) slave_config.conf & \
		sleep 0.5; \
	done
	@echo "所有从节点已启动"

# 特殊目标
debug: BUILD = debug
debug: all

release: BUILD = release
release: all

# 仅构建 OpenCV 程序
opencv_only: $(TARGET)

# 仅构建 ZeroMQ 程序
zmq_only: $(ZMQ_MASTER_TARGET) $(ZMQ_SLAVE_TARGET)

# 显示帮助信息
help:
	@echo "可用命令:"
	@echo "  make all              - 编译所有程序 (OpenCV + ZeroMQ)"
	@echo "  make opencv_only      - 仅编译 OpenCV 主程序"
	@echo "  make zmq_only         - 仅编译 ZeroMQ 程序"
	@echo "  make debug            - 编译调试版本"
	@echo "  make release          - 编译发布版本"
	@echo "  make clean            - 清理编译文件"
	@echo "  make install_deps     - 安装依赖库"
	@echo "  make create_configs   - 创建示例配置文件"
	@echo ""
	@echo "运行命令:"
	@echo "  make run_opencv       - 运行 OpenCV 主程序"
	@echo "  make run_master       - 运行 ZeroMQ 主节点"
	@echo "  make run_slave id=N   - 运行指定 ZeroMQ 从节点"
	@echo "  make run_all_slaves   - 运行所有 ZeroMQ 从节点(1-9)"
	@echo ""
	@echo "目标说明:"
	@echo "  $(TARGET)          - OpenCV AprilTag 检测程序"
	@echo "  $(ZMQ_MASTER_TARGET) - ZeroMQ 九宫格主节点"
	@echo "  $(ZMQ_SLAVE_TARGET)  - ZeroMQ 从节点"

# 显示版本信息
version:
	@echo "多目标构建系统 v1.0"
	@echo "包含: OpenCV AprilTag 检测 + ZeroMQ 分布式监控"
	@echo "编译器: $(CXX)"
	@echo "编译选项: $(CXXFLAGS)"

.PHONY: all check_deps clean install_deps create_configs \
        run_opencv run_master run_slave run_all_slaves \
        debug release opencv_only zmq_only help version