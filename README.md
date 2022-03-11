- [cn](#操作指南)
- [en](#Instructions)
# 操作指南

> 此SDK仅适用于深圳乐动机器人有限公司销售的激光雷达产品，产品型号为:
> - LDROBOT LiDAR  LD14.

## 1. 设备权限设置
- 可以通过`ls -l /dev/`查看雷达对应在系统中的挂载
> 例如：如果雷达在你的系统上挂载为`/dev/ttyS0`,则：
``` bash
sudo chmod 777 /dev/ttyS0
```

## 2. 编译

```bash
mkdir build # 若工程目录下不存在build文件夹则需创建
cd build
cmake ../
make
```

## 3. 运行
- 产品为 LDROBOT LiDAR LD14

  ``` bash
  ./ldlidar LD14 <serial_number>
  # eg ./ldlidar LD14 /dev/ttyS0
  ```

# Instructions

> This SDK is only applicable to the LiDAR products sold by Shenzhen LDROBOT Co., LTD. The product models are LDROBOT LiDAR LD00, LDROBOT LiDAR LD03,LDROBOT LiDAR LD08, LDROBOT LiDAR LD14.

## step 1: Device Permission Settings

- You can use `ls -l /dev/` to view the radar mounted on the system  
> For example, if the radar is mounted on your system as `/dev/ttys0`, then:  
``` bash
sudo chmod 777 /dev/ttyS0
```

## step 2: build

``` bash
mkdir build # If the build folder does not exist in the project directory, create one
cd build
cmake ../
make
```

## step 3: run

- The product is LDROBOT LiDAR LD14

  ``` bash
  ./ldlidar LD14 <serial_number>
  # eg ./ldlidar LD14 /dev/ttyS0
  ```
