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

# 目标设置
DIR_INC	:= ./include/
TARGET := opencv_program
SRC := main.cpp
OBJ := $(SRC:.cpp=.o)

# 构建模式（默认debug）
BUILD ?= debug

# 模式特定配置
ifeq ($(BUILD),release)
	CXXFLAGS += -O3 -DNDEBUG
else
	CXXFLAGS += -g -O0
endif

# 默认目标
all: $(TARGET)

# 主目标链接
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(OPENCV_LIBS) $(APRILTAG_LIBS)

# 编译规则
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(OPENCV_FLAGS) $(APRILTAG_INC)  -c $< -o $@

# 清理
clean:
	rm -f $(OBJ) $(TARGET)

# 特殊目标
debug: BUILD = debug
debug: all

release: BUILD = release
release: all

.PHONY: all clean debug release
