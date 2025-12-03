#!/usr/bin/env python3
"""
GPIO Shutdown Monitor for RDK X5
Monitors GPIO pin 37 (BOARD) and shuts down system when LOW level detected
低电平触发关机版本
"""

import sys
import signal
import os
import logging
import subprocess
import time
import Hobot.GPIO as GPIO

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('/var/log/gpio-shutdown.log')
    ]
)
logger = logging.getLogger('GPIO-Shutdown-Monitor')

# 定义使用的GPIO通道
INPUT_PIN = 37  # BOARD 编码 37

def signal_handler(signal, frame):
    """处理中断信号"""
    logger.info("Received interrupt signal, cleaning up...")
    GPIO.cleanup()
    sys.exit(0)

def shutdown_system():
    """执行关机操作"""
    logger.info("LOW level detected! Initiating system shutdown...")
    
    try:
        # 使用subprocess执行关机
        result = subprocess.run(
            ["sudo", "shutdown", "-h", "now"],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode == 0:
            logger.info("Shutdown command executed successfully")
        else:
            logger.error(f"Shutdown command failed: {result.stderr}")
            
    except subprocess.TimeoutExpired:
        logger.error("Shutdown command timed out")
    except Exception as e:
        logger.error(f"Error executing shutdown: {e}")
        
    finally:
        GPIO.cleanup()
        sys.exit(0)

def main():
    """主函数"""
    prev_value = None
    shutdown_triggered = False
    low_level_duration = 0  # 记录低电平持续时间
    
    # 设置GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(INPUT_PIN, GPIO.IN)
    
    # 可选：启用内部上拉电阻（如果支持）
    # 这样可以确保引脚在未连接时保持高电平
    try:
        GPIO.setup(INPUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        logger.info("Internal pull-up resistor enabled")
    except:
        logger.info("Pull-up resistor not available or not needed")
    
    logger.info(f"GPIO Shutdown Monitor started. Monitoring pin {INPUT_PIN}")
    logger.info("Press CTRL+C to exit. System will shutdown when LOW level detected.")
    logger.info(f"Current pin state: {'HIGH' if GPIO.input(INPUT_PIN) else 'LOW'}")
    
    try:
        while True:
            # 读取管脚电平
            value = GPIO.input(INPUT_PIN)
            
            # 检查是否为低电平
            if value == GPIO.LOW:
                low_level_duration += 1
                logger.debug(f"Low level detected, duration: {low_level_duration} cycles")
                
                # 可选：添加防抖动，持续低电平超过N个周期才触发
                if low_level_duration >= 2 and not shutdown_triggered:  # 约1秒（0.5秒×2）
                    logger.info(f"Pin {INPUT_PIN} state: LOW (stable for {low_level_duration*0.5:.1f}s)")
                    shutdown_triggered = True
                    shutdown_system()
            else:
                # 高电平，重置计数器
                if low_level_duration > 0:
                    logger.info(f"Pin {INPUT_PIN} state changed to HIGH, resetting counter")
                low_level_duration = 0
            
            # 记录状态变化（用于调试）
            if value != prev_value:
                state_str = "LOW" if value == GPIO.LOW else "HIGH"
                logger.info(f"Pin {INPUT_PIN} state changed to {state_str}")
                prev_value = value
            
            time.sleep(0.5)  # 检测间隔
            
    except KeyboardInterrupt:
        logger.info("Program interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        GPIO.cleanup()
        logger.info("GPIO cleanup completed, exiting...")

if __name__ == '__main__':
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    main()
