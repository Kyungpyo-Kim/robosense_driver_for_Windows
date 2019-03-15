/**

	@mainpage Project: RSLIDAR 드라이버

	@section intro
		- Brief: RS LIDAR Driver for Windows(MSVS)
		- Details: 

	@section Program
	    - robosense_demo_cw

	@section InOut 
		- INPUT: LiDAR data via ethernet
		- OUTPUT: Point cloud (PCL XYZI type)

	@section  CreateInfo 
		- Author: kyungpyo.kim@control-works.co.kr
		- date: 2019/03/08

	@section  ModifyInfo
		- 2019/03/08
			-# Doxygen 문서화

	@section Caution
		-

	@subsection exec 
		- README.MD 참고

	@section common 
		- Control-works

*/

/////////////////////////////////////////////////////////////////
#pragma once

// C
#include <stdio.h> // to use fopen, fscan and so on, use _CRT_SECURE_NO_WARNINGS in preprocessors

// STD
#include <iostream>
#include <string>
#include <thread>
#include <mutex>

//BOOST
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

// CUSTOM
#include "structure_constant.h"
#include "sync_queue.hpp"


/**
 *
 * @brief robosense lidar driver
 * @details drver 및 API 제공
 * @see RS16, RS32 드라이버 검증 완료
 * @todo difop_port recieve 함수 디버깅 및 검증 필요
 * @author kyungpyo.kim@control-works.co.kr
 * @date 2019-03-08
 *
 */
class rslidar_driver_cw
{

public:
	rslidar_driver_cw(
		std::string model,
		std::string address = std::string("192.168.1.200"), // default value
		unsigned short msop_port = 6699, // default value
		unsigned short difop_port = 1111,
		double cut_angle = -0.01,
		double rpm = 600.0); // default value

	~rslidar_driver_cw();

public:
	/**
	 * @brief API - load configuration parameters from csv files
	 * @details from ros ROS RSLIDAR DRIVER
	 * @param std::string anglePath
	 * @param std::string curvesPath
	 * @param std::string channelPath
	 * @param std::string curvesRatePath
	 * @param std::string model
	 * @return bool
	 * @throws 
	 */
	bool loadConfigFile(
		std::string anglePath,
		std::string curvesPath,
		std::string channelPath,
		std::string curvesRatePath,
		std::string model);

	/**
	* @brief API - start lidar driver
	* @details 
	* @param 
	* @return bool
	* @throws
	*/
	bool start_driver(void);

	/**
	* @brief API - run lidar driver
	* @details
	* @param
	* @return 
	* @throws
	*/
	void run_driver(void);

	/**
	* @brief API - wait lidar driver
	* @details locate on the end of application
	* @param
	* @return 
	* @throws
	*/
	void wait_driver(void);

	/**
	* @brief API - get point cloud
	* @details 
	* @param pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud
	* @return bool
	* @throws
	*/
	bool get_point_cloud(
		pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud);


private:
	// thread
	void running_io_service_thread(void);
	void lidar_scan_processing_thread(void);
	void point_clooud_parsing_thread(void);

private:
	// for socket communication
	boost::asio::io_service io_service_;
	boost::asio::ip::address address_;
	unsigned short msop_port_;
	unsigned short difop_port_;

	boost::shared_ptr<boost::asio::ip::udp::socket> msop_socket_;
	boost::shared_ptr<boost::asio::ip::udp::socket> difop_socket_;
	boost::asio::ip::udp::endpoint msop_remote_endpoint_;
	boost::asio::ip::udp::endpoint difop_remote_endpoint_;
	
	uint8_t msop_buffer_[PACKET_SIZE];
	uint8_t difop_buffer_[PACKET_SIZE];
	
	bool get_one_scan_packet(std::vector<std::vector<uint8_t>>& scan);
	void get_difop_param(const std::vector<uint8_t>& data);

	void msop_packet_receive(void);
	void msop_handle_receive(const boost::system::error_code& error,
		std::size_t bytes_transferred);

	void difop_packet_receive(void);
	void difop_handle_receive(const boost::system::error_code& error,
		std::size_t bytes_transferred);

private:
	std::mutex mutex_pointcloud_;
	pcl::PointCloud<pcl::PointXYZI> pointcloud_;
	sync_queue<std::vector<uint8_t>> packet_que_;
	sync_queue<std::vector<std::vector<uint8_t>>> scan_que_;
	std::vector<std::thread> t_vec_;


/// ROS RSLIDAR DRIVER CODE
/// https://github.com/RoboSense-LiDAR/ros_rslidar
/// ROS RSLIDAR DRIVER CODE
private:
	
	struct
	{
		std::string model;     ///< device model name
		int npackets;          ///< number of packets to collect
		double rpm;            ///< device rotation rate (RPMs)
		int cut_angle;
	} config_;

	int numOfLasers_;
	double TEMPERATURE_RANGE_;
	double intensityFactor_;
	double intensity_mode_;
	bool Curvesis_new_;
	float aIntensityCal_[7][32];
	float aIntensityCal_old_[1600][32];
	float VERT_ANGLE_[32];
	float HORI_ANGLE_[32];
	int g_ChannelNum_[32][51];
	float CurvesRate_[32];
	bool is_init_curve_;
	bool is_init_angle_;
	int block_num_;
	int tempPacketNum_;
	float temper_;

public:
	void unpack(const std::vector<uint8_t>& data, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud);
	void unpack_RS32(const std::vector<uint8_t>& data, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud);
	float calibrateIntensity_old(float intensity, int calIdx, int distance);
	float calibrateIntensity(float intensity, int calIdx, int distance);
	float pixelToDistance(int pixelValue, int passageway);
	int estimateTemperature(float Temper);
	float computeTemperature(unsigned char bit1, unsigned char bit2);
	int isABPacket(int distance);
	int correctAzimuth(float azimuth_f, int passageway);
};
