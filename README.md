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

### 增加关机按键监控

#### 0. 填写配置文件

sudo nano /etc/systemd/system/gpio-shutdown.service

```
[Unit]
Description=GPIO Shutdown Monitor for RDK X5 (LOW level trigger)
After=multi-user.target
Wants=network-online.target
Requires=syslog.service
StartLimitIntervalSec=0

[Service]
Type=simple
User=root
ExecStart=/usr/bin/python3 /usr/local/bin/gpio-shutdown-monitor.py
Restart=always
RestartSec=10
StandardOutput=syslog
StandardError=syslog
SyslogIdentifier=gpio-shutdown

# 安全设置
PrivateTmp=true
NoNewPrivileges=true
ProtectSystem=strict
ProtectHome=true
ReadWritePaths=/var/log/

[Install]
WantedBy=multi-user.target
```

#### 0. 复制脚本到root目录
sudo cp gpio-shutdown-monitor.py /usr/local/bin/gpio-shutdown-monitor.py

#### 1. 重新加载systemd配置
sudo systemctl daemon-reload

#### 2. 重启服务
sudo systemctl restart gpio-shutdown.service

#### 3. 查看服务状态
sudo systemctl status gpio-shutdown.service

#### 4. 查看实时日志
sudo journalctl -u gpio-shutdown.service -f
