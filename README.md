High FPS (120FPS) apriltag detect

### Add autostart service

#### 0. 填写配置文件
sudo nano /etc/systemd/system/opencv-program.service

```
[Unit]
Description=OpenCV Program AutoStart
After=network.target

[Service]
Type=simple
User=sunrise
WorkingDirectory=/home/sunrise/apriltag_detect_cpp/
ExecStart=/home/sunrise/apriltag_detect_cpp/opencv_program --run 0
Restart=on-failure
StandardOutput=truncate:/home/sunrise/opencv_program.log
StandardError=inherit

[Install]
WantedBy=multi-user.target

```

#### 1. 重载systemd配置，使其识别新服务
sudo systemctl daemon-reload

#### 2. 设置服务开机自启动
sudo systemctl enable opencv-program.service

#### 3. 立即启动服务进行测试
sudo systemctl start opencv-program.service

#### 4. 检查服务运行状态
sudo systemctl status opencv-program.service
