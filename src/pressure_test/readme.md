# 压力测试步骤
## 1. 将激光雷达接入ubuntu系统,设置对应USB设备权限
```bash
# eg: /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB0
```
## 2. 编译压力测试程序
```bash
./auto_build.sh
```
## 3. GDB模式下执行压测程序
```bash
cd build 

gdb ldlidar_sl   # 进入gdb模式

r  # 在gdb模式下运行程序

Ctrl + C #中断程序

q # 退出gdb模式

```