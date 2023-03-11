# 1.系统设置

- 第一步，通过板载串口或者USB转串口模块(例如,cp2102模块)的方式,使激光雷达连接到你的系统主板.
- 第二步，设置激光雷达在系统中挂载的串口设备-x权限,根据激光雷达在你的系统中的实际挂载情况来设置，可以使用`ls -l /dev`命令查看,(以/dev/ttyUSB0为例).

``` bash
$ cd ~/ldlidar_ws/ldlidar_sdk/sample/linux

$ sudo chmod 777 /dev/ttyUSB0
```

# 2.编译

```bash
$ cd ~/ldlidar_ws/ldlidar_sdk/sample/linux

$ ./auto_build.sh
```

# 3.运行
``` bash
$ ./build/ldlidar <lidar_typename> <serial_number>
# example:
# LDLiDAR LD14 
$ ./build/ldlidar LD14 /dev/ttyUSB0
# LDLiDAR LD14P 
$ ./build/ldlidar LD14P /dev/ttyUSB0
# LDLiDAR LD06
$ ./build/ldlidar LD06 /dev/ttyUSB0
# LDLiDAR LD19
$ ./build/ldlidar LD19 /dev/ttyUSB0
```