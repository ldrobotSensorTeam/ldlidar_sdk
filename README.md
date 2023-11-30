# LDLiDAR SDK - Fixed

## Fixes by MrKussen including:

- Deleted non-working code
- Added missing functions
- Fixing includes
- Added one missing header guard instead of pragma once, to make the code uniform.
- Added platform-dependant defines to decrease confusion and making sure that only code for the right platform will be compiled and run.
- The linux code was mostly fine, most changes were either in the windows code or the logging module

## How to use (mostly for beginners):

You can use this SDK by just adding the cpp files and the headers to your projects. 

When you compile, there might be errors if you have UNICODE or _UNICODE defined, so avoid defining those if you can, or you will have to change between wide and thin characters in the source code. 

Make sure that you compile the .cpp files here as well as your program files. 

(For example, compilation with g++ can be done by placing your program files in /src/ and running g++ ./src/*.cpp -I./include/)

---

## Supported products

This SDK is only applicable to the LiDAR products sold by Shenzhen LDROBOT Co., LTD. 
|    product models      |  range types |
|     ----               |  ----        |
|   LDROBOT LiDAR LD06   | 2D DTOF      |
|   LDROBOT LiDAR LD19   | 2D DTOF      |
|   LDROBOT LiDAR LD14   | 2D Triangle  |
|   LDROBOT LiDAR LD14P  | 2D Triangle  |

---
## Get SDK
- download package file or use git.
```bash
$ cd ~

$ mkdir  ldlidar_ws

$ cd ldlidar_ws

$ git clone  https://github.com/ldrobotSensorTeam/ldlidar_sdk.git
```

---
## Introduction to linux application example
- [EN](./sample/linux/README.md)
- [CN](./sample/linux/README_CN.md)

---

## Introduction to windows application example
- [EN](./sample/windows/README.md)
- [CN](./sample/windows/README_CN.md)
