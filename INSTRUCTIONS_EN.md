## get SDK sound code
```bash
$ cd ~

$ mkdir  ldlidar_ws

$ cd ldlidar_ws

$ git clone  https://github.com/ldrobotSensorTeam/ldlidar_sdk.git
```
## system setup
- Connect the LiDAR to your system motherboard via an onboard serial port or usB-to-serial module (for example, CP2102 module).

- Set the -x permission for the serial port device mounted by the lidar in the system (for example, /dev/ttyUSB0),In actual use, the LiDAR can be set according to the actual mounted status of your system, you can use 'ls -l /dev' command to view.

``` bash
$ cd ~/ldlidar_ws/ldlidar_sdk

$ sudo chmod 777 /dev/ttyUSB0
```

## build

``` bash
$ cd ~/ldlidar_ws/ldlidar_sdk
$ ./auto_build.sh
```

## run
``` bash
$ ./build/ldlidar <lidar_typename> <serial_number>
# example:
# LDLiDAR LD14 
$ ./build/ldlidar LD14 /dev/ttyUSB0
# LDLiDAR LD06
$ ./build/ldlidar LD06 /dev/ttyUSB0
# LDLiDAR LD19
$ ./build/ldlidar LD19 /dev/ttyUSB0
```