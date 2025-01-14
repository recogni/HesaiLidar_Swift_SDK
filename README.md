# HesaiLidar_Swift_SDK
## About the project
This repository includes the software development kit for Pandar LiDAR sensor manufactured by Hesai Technology. Branches are included for different systems and UDP protocol versions:
* master: The software development kit for Ubuntu 18.04 and Ubuntu 20.04 supports Hesai lidar with UDP protocol v1.3, v1.4, v3.2 
* ubuntu16.04: The software development kit for Ubuntu 16.04 supports Hesai lidar with UDP protocol v1.3, v1.4, v3.2  

LiDAR sensor manufactured by Hesai Technology.
## Environment and Dependencies

**System environment requirement:Linux**
```
Recommanded
-Ubuntu 16.04
-Ubuntu 18.04
-Ubuntu 20.04
```

**Compiler vresion requirement**
```
Cmake version requirement:Cmake 3.8.0 or above
G++ version requirement:G++ 7.5 or above
```
**Library Dependencies: libpcl-dev + libpcap-dev + libyaml-cpp-dev + libboost-dev**
[Add libzstd-dev: This was not installed on AGX]
```
$ sudo apt install libyaml-cpp-dev libboost-dev
$ sudo apt install libzstd-dev
$ sudo apt install libpcl-dev libpcap-dev
```

## Clone
```
$ git clone https://github.com/HesaiTechnology/HesaiLidar_Swift_SDK.git
```

## Build
```
1.$ cd HesaiLidar_Swift_SDK
2.$ mkdir build
3.$ cd build
4.$ cmake ..
5.$ make
```

## Add to your project
### Cmake
```
add_subdirectory(<path_to>HesaiLidar_Swift_SDK)

include_directories(
	<path_to>HesaiLidar_Swift_SDK/include
	<path_to>HesaiLidar_Swift_SDK/src
)

target_link_libraries(<Your project>
  PandarSwiftSDK
)
```

## Run

Set the parameters of class PandarSwiftSDK in test.cc
```
// for Pandar128
PandarSwiftSDK(std::string("192.168.1.201"), 2368, 10110, std::string("Pandar128"), \
                                std::string("../params/Pandar128_Correction.csv"), \
                                std::string("../params/Pandar128_Firetimes.csv"), \
                                std::string(""), \
                                lidarCallback, rawcallback, gpsCallback, \
                                std::string(""), \
                                std::string(""), \
                                std::string(""), \
                                0, 0, std::string("both_point_raw"), false);


// for PandarQT128
PandarSwiftSDK(std::string("192.168.1.201"), 2368, 10110, std::string("PandarQT128"), \
                                std::string("../params/PandarQT128_Correction.csv"), \
                                std::string("../params/PandarQT128_Firetimes.csv"), \
                                std::string(""), \
                                lidarCallback, rawcallback, gpsCallback, \
                                std::string(""), \
                                std::string(""), \
                                std::string(""), \
                                0, 0, std::string("both_point_raw"), false);

```
Parameter description
```
        deviceipaddr  	  The ip of the device
        lidarport 	      The port number of lidar data
        gpsport           The port number of gps data
        frameid           The id of the point cloud data published to ROS
        correctionfile    The correction file path
        firtimeflie       The firtime flie path
        pcapfile          The pcap flie path
        pclcallback       The callback of PCL data structure
        rawcallback       The callback of raw data structure
        gpscallback       The callback of GPS structure
        certFile          The path of the user's certificate
        privateKeyFile    The path of the user's private key
        caFile            The path of the root certificate
        start_angle       The start angle of every point cloud should be <real angle> * 100.
        timezone          The timezone of local
        publishmode       The mode of publish
        datatype          The model of input data
coordinateCorrectionFlag  The flag to control whether to do coordinate Correction

```
Set the pcap flie path only when you what to read a pcap
```
$ make 
$ ./PandarSwiftTest
```

## Inititial Set up
Check status of the firewall:
```
sudo ufw status
```

Disable firewall for certain ports and services
### Allow ptpd
```
sudo ufw allow ptpd
```

### Allow ports for lidar communication
TCP/9347 (command port) and UDP/2368 (data port)
```
sudo ufw allow 9347/tcp comment 'Open Lidar control port'
sudo ufw allow 2368/udp comment 'Open Lidar data port'
```

### View all added rules
```
sudo ufw show added
```



## Fine tuning
To avoid packet drops, adjust the buffer size:
sudo sysctl --write net.core.rmem_max=52428800
sudo sysctl --write net.core.rmem_default=52428800

And, the buffer size in the code is already large enough.

## About timestamps
Currently we are using PTP server based UTC timestamps for lidar.

Argus on Jetson NX reports time since boot.  In addition, there is an extra offset of 7307464 seconds
(computed empirically). Need to get a precise value of this offset in order to fully synchronize the
Lidar PCL and raw sensor images.

Take TZ into account (convert to PST, probably)
Note that 
```
date -d "$(who -b | awk '{print $4,$3}' | tr - / )" +%s
```
gives timestamp when system was booted up

```
date +%s
```
give current system timestamp (seconds since epoch)
