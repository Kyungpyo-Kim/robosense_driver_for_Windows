# Robosense LiDAR Driver for Windows OS

## Environment
* Windows10 64bit
* Visual Studio 2017 Express

## Dependencies
* Boost
* PCL

## API 사용법
* Class object 선언

```cpp
rslidar_driver_cw rd;
```

* Load Configuration files

```cpp
std::string anglePath("cfg/configuration_data/angle.csv");
std::string curvesPath("cfg/configuration_data/curves.csv");
std::string channelPath("cfg/configuration_data/ChannelNum.csv");
std::string curvesRatePath("");
std::string model("RS32");

// 1. load configuration
rd.loadConfigFile(
	anglePath,
	curvesPath,
	channelPath,
	curvesRatePath,
	model);
```

* Start driver

```cpp
// 2. start driver
rd.start_driver();
```

* Run driver

```cpp
// 3. run driver (thread)
rd.run_driver();
```

* Get point cloud using API

```cpp
// 99. run algorithm using "get_point_cloud" function
// --------------------------------------------
// --------------------------------------------
// 99. Get point cloud data from driver
// --------------------------------------------
pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
while (!rd.get_point_cloud(outPoints));;
// --------------------------------------------
```

* Caution

```cpp
// Be careful!
// wait_driver() has to be located at the end of the algorithm(or main function)
// refer the join() of the STD Thread
rd.wait_driver();
```


## TODO
* 전처리기 추가
  - 속성 >> 구성속성 >> C/C++ >> 전처리기 >> 전처리기 정의 >> 편집 선택
  - **_CRT_SECURE_NO_WARNINGS** 추가

## Build
  - Doxygen 추가

## Visualization demo

* 16 채널
![demo_image](https://github.com/Kyungpyo-Kim/robosense_demo_cw/blob/master/vis_demo.PNG)

* 32 채널
![demo_image_32](https://github.com/Kyungpyo-Kim/robosense_demo_cw/blob/master/vis_demo_32.PNG)
