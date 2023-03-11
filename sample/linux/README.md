# 1.system setup
- Connect the LiDAR to your system motherboard via an onboard serial port or usB-to-serial module (for example, CP2102 module).

- Set the -x permission for the serial port device mounted by the lidar in the system (for example, /dev/ttyUSB0),In actual use, the LiDAR can be set according to the actual mounted status of your system, you can use 'ls -l /dev' command to view.

``` bash
$ cd ~/ldlidar_ws/ldlidar_sdk/sample/linux

$ sudo chmod 777 /dev/ttyUSB0
```

# 2.build

``` bash
$ cd ~/ldlidar_ws/ldlidar_sdk/sample/linux

$ ./auto_build.sh
```

# 3.run
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