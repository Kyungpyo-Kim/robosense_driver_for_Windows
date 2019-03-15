#include "rsdriver_cw.h"

#include <fstream>

rslidar_driver_cw::rslidar_driver_cw(
	std::string model,
	std::string address,
	unsigned short msop_port,
	unsigned short difop_port,
	double cut_angle,
	double rpm)
	:
	address_(boost::asio::ip::address::from_string(address)), 
	msop_port_(msop_port),
	difop_port_(difop_port),
	block_num_(0),
	tempPacketNum_(0),
	temper_(31.0),
	is_init_curve_(false),
	is_init_angle_(false),
	msop_socket_(NULL),
	difop_socket_(NULL) {

	// get model name, validate string, determine packet rate
	config_.model = model;
	double packet_rate;  // packet frequency (Hz)
	std::string model_full_name;

	// product model
	if (config_.model == "RS16")
	{
		packet_rate = 840;
		model_full_name = "RS-LiDAR-16";
	}
	else if (config_.model == "RS32")
	{
		packet_rate = 1690;
		model_full_name = "RS-LiDAR-32";
	}
	else
	{
		std::cerr << "unknown LIDAR model: " << config_.model << std::endl;
		packet_rate = 2600.0;
	}
	std::string deviceName(std::string("Robosense ") + model_full_name);

	config_.rpm = rpm;
	double frequency = (config_.rpm / 60.0);  // expected Hz rate

	// default number of packets for each scan is a single revolution
	// (fractions rounded up)

	int npackets = (int)ceil(packet_rate / frequency);
	config_.npackets = npackets;
	std::cout << "Publishing " << config_.npackets << " packets per scan" << std::endl;

	int msop_udp_port;
	msop_udp_port = (int)MSOP_DATA_PORT_NUMBER;
	int difop_udp_port;
	difop_udp_port = (int)DIFOP_DATA_PORT_NUMBER;

	cut_angle = -0.01;
	if (cut_angle < 0.0)
	{
		std::cout << "Cut at specific angle feature deactivated." << std::endl;
	}
	else if (cut_angle < 360)
	{
		std::cout << "Cut at specific angle feature activated. " << std::endl
			<< "Cutting rslidar points always at " << std::endl
			<< cut_angle << " degree." << std::endl;
	}
	else
	{
		std::cerr << "cut_angle parameter is out of range. Allowed range is " << std::endl
			<< "between 0.0 and 360 negative values to deactivate this feature." << std::endl;
		cut_angle = -0.01;
	}

	// Convert cut_angle from radian to one-hundredth degree,
	// which is used in rslidar packets
	config_.cut_angle = static_cast<int>(cut_angle * 100);


	// Open sockets
	try {

		msop_socket_.reset(
			new boost::asio::ip::udp::socket(
				io_service_, 
				boost::asio::ip::udp::endpoint(address_, msop_port_)));
	}
	catch (const std::exception& ex) {

		try {
			msop_socket_.reset(
				new boost::asio::ip::udp::socket(
					io_service_, 
					boost::asio::ip::udp::endpoint(
						boost::asio::ip::udp::v4(), 
						msop_port_)));
			std::cout << "msop socket open without address" << ex.what() << std::endl;
		}
		catch (const std::exception& exx) {

			std::cerr << "msop socket open error: " << exx.what() << std::endl;
		}
	}
	try {

		difop_socket_.reset(
			new boost::asio::ip::udp::socket(
				io_service_,
				boost::asio::ip::udp::endpoint(address_, difop_port_)));
	}
	catch (const std::exception& ex) {

		try {
			difop_socket_.reset(
				new boost::asio::ip::udp::socket(
					io_service_,
					boost::asio::ip::udp::endpoint(
						boost::asio::ip::udp::v4(),
						difop_port_)));
			std::cout << "difop socket open without address" << ex.what() << std::endl;
		}
		catch (const std::exception& exx) {

			std::cerr << "difop socket open error: " << exx.what() << std::endl;
		}
	}
}

rslidar_driver_cw::~rslidar_driver_cw() {}


// 2019-03-06 revision by kyungpyo.kim@control-works.co.kr
/// revision 2019-02-28 by kyungpyo.kim@control-works.co.kr
bool rslidar_driver_cw::loadConfigFile(
	std::string anglePath,
	std::string curvesPath,
	std::string channelPath,
	std::string curvesRatePath,
	std::string model) {

	if (model == std::string("RS16"))
	{
		numOfLasers_ = 16;
	}
	else if (model == "RS32")
	{
		numOfLasers_ = 32;
		TEMPERATURE_RANGE_ = 50;
	}

	intensityFactor_ = 51;
	intensity_mode_ = 1;

	/// 读参数文件 2017-02-27
	/// revision 2019-02-28 by kyungpyo.kim@control-works.co.kr
	FILE* f_inten = fopen(curvesPath.c_str(), "r");
	int loopi = 0;
	int loopj = 0;
	int loop_num;
	if (!f_inten)
	{
		std::cerr << curvesPath << " does not exist" << std::endl;
		return false;
	}
	else
	{
		fseek(f_inten, 0, SEEK_END);  //定位到文件末
		int size = ftell(f_inten);    //文件长度
		//std::cout << "size is::::::::::::::::::::::::::::: " << size;
		if (size > 10000)  //老版的curve
		{
			Curvesis_new_ = false;
			loop_num = 1600;
		}
		else
		{
			Curvesis_new_ = true;
			loop_num = 7;
		}
		fseek(f_inten, 0, SEEK_SET);
		while (!feof(f_inten))
		{
			float a[32];
			loopi++;

			if (loopi > loop_num)
				break;
			if (numOfLasers_ == 16)
			{
				int tmp = fscanf(f_inten, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", &a[0], &a[1], &a[2], &a[3],
					&a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12], &a[13], &a[14], &a[15]);
			}
			else if (numOfLasers_ == 32)
			{
				int tmp = fscanf(f_inten, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"
					"%f,%f,%f,%f\n",
					&a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12],
					&a[13], &a[14], &a[15], &a[16], &a[17], &a[18], &a[19], &a[20], &a[21], &a[22], &a[23], &a[24],
					&a[25], &a[26], &a[27], &a[28], &a[29], &a[30], &a[31]);
			}
			if (Curvesis_new_)
			{
				for (loopj = 0; loopj < numOfLasers_; loopj++)
				{
					aIntensityCal_[loopi - 1][loopj] = a[loopj];
				}
			}
			else
			{
				for (loopj = 0; loopj < numOfLasers_; loopj++)
				{
					aIntensityCal_old_[loopi - 1][loopj] = a[loopj];
				}
			}
			// ROS_INFO_STREAM("new is " << a[0]);
		}
		fclose(f_inten);
	}
	//=============================================================
	FILE* f_angle = fopen(anglePath.c_str(), "r");
	if (!f_angle)
	{
		std::cerr << anglePath << " does not exist" << std::endl;
		return false;
	}
	else
	{
		float b[32], d[32];
		int loopk = 0;
		int loopn = 0;
		while (!feof(f_angle))
		{
			int tmp = fscanf(f_angle, "%f,%f\n", &b[loopk], &d[loopk]);
			loopk++;
			if (loopk > (numOfLasers_ - 1))
				break;
		}
		for (loopn = 0; loopn < numOfLasers_; loopn++)
		{
			VERT_ANGLE_[loopn] = b[loopn] / 180 * (float)PI;
			HORI_ANGLE_[loopn] = d[loopn] * 100;
		}
		fclose(f_angle);
	}

	//=============================================================
	FILE* f_channel = fopen(channelPath.c_str(), "r");
	if (!f_channel)
	{
		std::cerr << channelPath << " does not exist" << std::endl;
		return false;
	}
	else
	{
		int loopl = 0;
		int loopm = 0;
		int c[51];
		int tempMode = 1;
		while (!feof(f_channel))
		{
			if (numOfLasers_ == 16)
			{
				int tmp = fscanf(f_channel,
					"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%"
					"d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
					&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12],
					&c[13], &c[14], &c[15], &c[16], &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24],
					&c[25], &c[26], &c[27], &c[28], &c[29], &c[30], &c[31], &c[32], &c[33], &c[34], &c[35], &c[36],
					&c[37], &c[38], &c[39], &c[40]);
			}
			else
			{
				int tmp = fscanf(
					f_channel, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%"
					"d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
					&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12], &c[13],
					&c[14], &c[15], &c[16], &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24], &c[25], &c[26],
					&c[27], &c[28], &c[29], &c[30], &c[31], &c[32], &c[33], &c[34], &c[35], &c[36], &c[37], &c[38], &c[39],
					&c[40], &c[41], &c[42], &c[43], &c[44], &c[45], &c[46], &c[47], &c[48], &c[49], &c[50]);
			}
			//                if (c[1] < 100 || c[1] > 3000)
			//                {
			//                    tempMode = 0;
			//                }
			for (loopl = 0; loopl < TEMPERATURE_RANGE_ + 1; loopl++)
			{
				g_ChannelNum_[loopm][loopl] = c[tempMode * loopl];
			}
			loopm++;
			if (loopm > (numOfLasers_ - 1))
			{
				break;
			}
		}
		fclose(f_channel);
	}

	if (numOfLasers_ == 32)
	{
		FILE* f_curvesRate = fopen(curvesRatePath.c_str(), "r");
		if (!f_curvesRate)
		{
			std::cerr << curvesRatePath << " does not exist" << std::endl;
			return false;
		}
		else
		{
			int loopk = 0;
			while (!feof(f_curvesRate))
			{
				int tmp = fscanf(f_curvesRate, "%f\n", &CurvesRate_[loopk]);
				loopk++;
				if (loopk > (numOfLasers_ - 1))
					break;
			}
			fclose(f_curvesRate);
		}
	}

	return true;
}


// 2019-03-06 revision by kyungpyo.kim@control-works.co.kr
/// 2019-03-05 initialization by kyungpyo.kim@control-works.co.kr
bool rslidar_driver_cw::start_driver(void) {

	std::cout << "------------------------------------------" << std::endl;
	std::cout << "------------------------------------------" << std::endl;
	std::cout << "Start RS LiDAR Driver" << std::endl;
	std::cout << "model: " << config_.model << std::endl;
	std::cout << "IP address: " << address_ << std::endl;
	std::cout << "difop port: " << difop_port_ << std::endl;
	std::cout << "msop port: " << msop_port_ << std::endl;
	std::cout << "------------------------------------------" << std::endl;
	std::cout << "------------------------------------------" << std::endl;
		
	msop_packet_receive();
	difop_packet_receive();

	return true;
}


// 2019-03-06 initialization by kyungpyo.kim@control-works.co.kr
void rslidar_driver_cw::run_driver(void) {
	
	std::cout << "------------------------------------------" << std::endl;
	std::cout << "- running_io_service_thread" << std::endl;
	std::cout << "- lidar_scan_processing_thread" << std::endl;
	std::cout << "- point_clooud_parsing_thread" << std::endl;
	std::cout << "------------------------------------------" << std::endl;

	t_vec_.push_back(std::thread(&rslidar_driver_cw::running_io_service_thread, this)); // for socket communication
	t_vec_.push_back(std::thread(&rslidar_driver_cw::lidar_scan_processing_thread, this)); // for getting lidar scan data
	t_vec_.push_back(std::thread(&rslidar_driver_cw::point_clooud_parsing_thread, this)); // for parsing scan data
}


// 2019-03-06 initialization by kyungpyo.kim@control-works.co.kr
void rslidar_driver_cw::wait_driver(void) {

	for (auto it = t_vec_.begin(); it != t_vec_.end(); ++it)
		it->join();
}


// 2019-03-06 initialization by kyungpyo.kim@control-works.co.kr
bool rslidar_driver_cw::get_point_cloud(
	pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud) {

	if (pointcloud_.empty()) {

		//std::cout << "already you got point cloud already" << std::endl;
		return false;
	}

	mutex_pointcloud_.lock();
	pcl::copyPointCloud(pointcloud_, *pointcloud);
	pointcloud_.clear();
	mutex_pointcloud_.unlock();

	return true;
}


// 2019-03-06 revision by kyungpyo.kim@control-works.co.kr
/// 2019-03-05 initialization by kyungpyo.kim@control-works.co.kr
void rslidar_driver_cw::running_io_service_thread() {

	try {
		io_service_.run();
	}
	catch (const std::exception& ex) {
		std::cerr << "running_io_service_thread error: " << ex.what() << std::endl;
		return;
	}
}


// 2019-03-06 initialization by kyungpyo.kim@control-works.co.kr
void rslidar_driver_cw::lidar_scan_processing_thread() {
	
	while (true) {
		
		std::vector<std::vector<uint8_t>> scan;
		if (get_one_scan_packet(scan))
			scan_que_.push(scan);
	}
}


// 2019-03-06 initialization by kyungpyo.kim@control-works.co.kr
void rslidar_driver_cw::point_clooud_parsing_thread(void) {

	while (true) {
		if (scan_que_.empty()) continue;

		std::vector<std::vector<uint8_t>> scan;
		scan_que_.pop(scan);

		pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);

		if (config_.model == "RS16") {

			outPoints->height = 16;
			outPoints->width = 24 * (int)scan.size();
			outPoints->is_dense = false;
			outPoints->resize(outPoints->height * outPoints->width);
		}
		else if (config_.model == "RS32") {

			outPoints->height = 32;
			outPoints->width = 12 * (int)scan.size();
			outPoints->is_dense = false;
			outPoints->resize(outPoints->height * outPoints->width);
		}

		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

		block_num_ = 0;
		for (size_t i = 0; i < scan.size(); ++i) {

			unpack(scan[i], outPoints);
		}

		mutex_pointcloud_.lock();
		pcl::copyPointCloud(*outPoints, pointcloud_);
		mutex_pointcloud_.unlock();
	}
}


// 2019-03-06 revision by kyungpyo.kim@control-works.co.kr
/// 2019-03-05 initialization by kyungpyo.kim@control-works.co.kr
void rslidar_driver_cw::msop_packet_receive(void) {

	try {
		msop_socket_->async_receive_from(
			boost::asio::buffer(msop_buffer_), 
			msop_remote_endpoint_,
			boost::bind(&rslidar_driver_cw::msop_handle_receive,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}
	catch (const std::exception& ex) {
		std::cerr << "msop_packet_receive error: " << ex.what() << std::endl;
	}
	
}


// 2019-03-06 initialization by kyungpyo.kim@control-works.co.kr
void rslidar_driver_cw::msop_handle_receive(
	const boost::system::error_code& error,
	std::size_t bytes_transferred) {

	if ((int)bytes_transferred == PACKET_SIZE) {
		std::vector<uint8_t> data(PACKET_SIZE);
		memcpy(&data[0], msop_buffer_, PACKET_SIZE);
		
		packet_que_.push(data);
	}
	
	msop_packet_receive();
}


// 2019-03-06 initialization by kyungpyo.kim@control-works.co.kr
void rslidar_driver_cw::difop_packet_receive(void) {

	try {
		difop_socket_->async_receive_from(
			boost::asio::buffer(difop_buffer_),
			difop_remote_endpoint_,
			boost::bind(&rslidar_driver_cw::difop_handle_receive,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}
	catch (const std::exception& ex) {
		std::cerr << "difop_packet_receive error: " << ex.what() << std::endl;
	}
}


// 2019-03-06 initialization by kyungpyo.kim@control-works.co.kr
void rslidar_driver_cw::difop_handle_receive(
	const boost::system::error_code& error,
	std::size_t bytes_transferred) {

	if ((int)bytes_transferred == PACKET_SIZE) {
		std::vector<uint8_t> data(PACKET_SIZE);
		memcpy(&data[0], difop_buffer_, PACKET_SIZE);
		
		get_difop_param(data);
	}

	difop_packet_receive();
}


// 2019-03-06 revision by kyungpyo.kim@control-works.co.kr
/// 2019-03-05 revision by kyungpyo.kim@control-works.co.kr
/// 2019-02-28 initialization by kyungpyo.kim@control-works.co.kr
bool rslidar_driver_cw::get_one_scan_packet(std::vector<std::vector<uint8_t>>& scan) {

	scan.clear();

	// Since the rslidar delivers data at a very high rate, keep
	// reading and publishing scans as fast as possible.
	if (config_.cut_angle >= 0)  // Cut at specific angle feature enabled
	{
		scan.reserve(config_.npackets);
		while (true) {

			std::vector<uint8_t> tmp_packet(PACKET_SIZE);
			packet_que_.pop(tmp_packet);
			scan.push_back(tmp_packet);

			static int ANGLE_HEAD = -36001;  // note: cannot be set to -1, or stack smashing
			static int last_azimuth = ANGLE_HEAD;

			int azimuth = 256 * tmp_packet[44] + tmp_packet[45];
			// int azimuth = *( (u_int16_t*) (&tmp_packet.data[azimuth_data_pos]));

			// Handle overflow 35999->0
			if (azimuth < last_azimuth) {

				last_azimuth -= 36000;
			}
			// Check if currently passing cut angle
			if (last_azimuth != ANGLE_HEAD && last_azimuth < config_.cut_angle && azimuth >= config_.cut_angle) {

				last_azimuth = azimuth;
				break;  // Cut angle passed, one full revolution collected
			}
			last_azimuth = azimuth;
		}
	}
	else {  // standard behaviour

		scan.resize(config_.npackets);

		for (int i = 0; i < config_.npackets; ++i) {

			// keep reading until full packet received
			packet_que_.pop(scan[i]);
		}
	}

	return true;
}


// 2019-03-06 initialization by kyungpyo.kim@control-works.co.kr
void rslidar_driver_cw::get_difop_param(const std::vector<uint8_t>& data){

	//std::cout << "Enter difop callback!" << std::endl;

	if (is_init_curve_)
	{
		// check header
		if (data[0] == 0xa5 && data[1] == 0xff && data[2] == 0x00 && data[3] == 0x5a)
		{
			bool curve_flag = true;
			// check difop reigon has beed flashed the right data
			if ((data[50] == 0x00 || data[50] == 0xff) && (data[51] == 0x00 || data[51] == 0xff) &&
				(data[52] == 0x00 || data[52] == 0xff) && (data[53] == 0x00 || data[53] == 0xff))
			{
				curve_flag = false;
			}

			// TODO check why rsview here no 32 laser, be more careful the new, old version
			// Init curves
			if (curve_flag)
			{
				unsigned char checkbit;
				int bit1, bit2;
				for (int loopn = 0; loopn < numOfLasers_; ++loopn)
				{
					// check the curves' parameter in difop
					checkbit = *(&data[0] + 50 + loopn * 15) ^ *(&data[0] + 50 + loopn * 15 + 1);
					for (int loopm = 1; loopm < 7; ++loopm)
					{
						checkbit = checkbit ^ (*(&data[0] + 50 + loopn * 15 + loopm * 2)) ^ (*(&data[0] + 50 + loopn * 15 + loopm * 2 + 1));
					}
					if (checkbit != *(&data[0] + 50 + loopn * 15 + 14))
					{
						return;
					}
				}
				for (int loopn = 0; loopn < numOfLasers_; ++loopn)
				{
					// calculate curves' parameters
					bit1 = static_cast<int>(*(&data[0] + 50 + loopn * 15));
					bit2 = static_cast<int>(*(&data[0] + 50 + loopn * 15 + 1));
					aIntensityCal_[0][loopn] = (bit1 * 256 + bit2) * 0.001;
					bit1 = static_cast<int>(*(&data[0] + 50 + loopn * 15 + 2));
					bit2 = static_cast<int>(*(&data[0] + 50 + loopn * 15 + 3));
					aIntensityCal_[1][loopn] = (bit1 * 256 + bit2) * 0.001;
					bit1 = static_cast<int>(*(&data[0] + 50 + loopn * 15 + 4));
					bit2 = static_cast<int>(*(&data[0] + 50 + loopn * 15 + 5));
					aIntensityCal_[2][loopn] = (bit1 * 256 + bit2) * 0.001;
					bit1 = static_cast<int>(*(&data[0] + 50 + loopn * 15 + 6));
					bit2 = static_cast<int>(*(&data[0] + 50 + loopn * 15 + 7));
					aIntensityCal_[3][loopn] = (bit1 * 256 + bit2) * 0.001;
					bit1 = static_cast<int>(*(&data[0] + 50 + loopn * 15 + 8));
					bit2 = static_cast<int>(*(&data[0] + 50 + loopn * 15 + 9));
					aIntensityCal_[4][loopn] = (bit1 * 256 + bit2) * 0.00001;
					bit1 = static_cast<int>(*(&data[0] + 50 + loopn * 15 + 10));
					bit2 = static_cast<int>(*(&data[0] + 50 + loopn * 15 + 11));
					aIntensityCal_[5][loopn] = -(bit1 * 256 + bit2) * 0.0001;
					bit1 = static_cast<int>(*(&data[0] + 50 + loopn * 15 + 12));
					bit2 = static_cast<int>(*(&data[0] + 50 + loopn * 15 + 13));
					aIntensityCal_[6][loopn] = (bit1 * 256 + bit2) * 0.001;
				}
				this->is_init_curve_ = true;
				std::cout << "this->is_init_curve_ = "
					<< "true!" << std::endl;
				Curvesis_new_ = true;
			}

			if ((data[290] != 0x00) && (data[290] != 0xff))
			{
				intensityFactor_ = static_cast<int>(*(&data[0] + 290));  // intensity factor introduced since than 20181115
				std::cout << intensityFactor_ << std::endl;
			}

			if ((data[291] == 0x00) || (data[291] == 0xff) || (data[291] == 0xa1))
			{
				intensity_mode_ = 1;  // mode for the top firmware lower than T6R23V8(16) or T9R23V6(32)
				std::cout << "intensity mode is 1" << std::endl;
			}
			else if (data[291] == 0xb1)
			{
				intensity_mode_ = 2;  // mode for the top firmware higher than T6R23V8(16) or T9R23V6(32)
				std::cout << "intensity mode is 2" << std::endl;
			}
		}
	}

	if (!this->is_init_angle_)
	{
		// check header
		if (data[0] == 0xa5 && data[1] == 0xff && data[2] == 0x00 && data[3] == 0x5a)
		{
			bool angle_flag = true;
			// check difop reigon has beed flashed the right data
			if ((data[1165] == 0x00 || data[1165] == 0xff) && (data[1166] == 0x00 || data[1166] == 0xff) &&
				(data[1167] == 0x00 || data[1167] == 0xff) && (data[1168] == 0x00 || data[1168] == 0xff))
			{
				angle_flag = false;
			}
			// angle
			if (angle_flag)
			{
				// TODO check the HORI_ANGLE
				int bit1, bit2, bit3, symbolbit;
				for (int loopn = 0; loopn < numOfLasers_; ++loopn)
				{
					if (loopn < 8 && numOfLasers_ == 16)
					{
						symbolbit = -1;
					}
					else
					{
						symbolbit = 1;
					}
					bit1 = static_cast<int>(*(&data[0] + 1165 + loopn * 3));
					bit2 = static_cast<int>(*(&data[0] + 1165 + loopn * 3 + 1));
					bit3 = static_cast<int>(*(&data[0] + 1165 + loopn * 3 + 2));
					VERT_ANGLE_[loopn] = (bit1 * 256 * 256 + bit2 * 256 + bit3) * symbolbit * 0.0001f / 180 * M_PI;
					// std::cout << VERT_ANGLE[loopn] << std::endl;
					// TODO
					HORI_ANGLE_[loopn] = 0;
				}
				this->is_init_angle_ = true;
				std::cout << "this->is_init_angle_ = "
					<< "true!" << std::endl;
			}
		}
	}
	//std::cout << "finish get_difop_param" << std::endl;
}


// revision 2019-02-28 by kyungpyo.kim@control-works.co.kr
void rslidar_driver_cw::unpack(const std::vector<uint8_t>& data, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud){

	if (numOfLasers_ == 32)
	{
		unpack_RS32(data, pointcloud);
		return;
	}
	float azimuth;  // 0.01 dgree
	float intensity;
	float azimuth_diff;
	float azimuth_corrected_f;
	int azimuth_corrected;

	const raw_packet_t* raw = (const raw_packet_t*)&data[42];

	for (int block = 0; block < BLOCKS_PER_PACKET; block++, this->block_num_++)  // 1 packet:12 data blocks
	{
		azimuth = (float)(256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2);

		if (UPPER_BANK != raw->blocks[block].header){
		
			//std::cout << "skipping RSLIDAR DIFOP packet" << std::endl;
			break;
		}

		if (tempPacketNum_ < 20000 && tempPacketNum_ > 0){  // update temperature information per 20000 packets

			tempPacketNum_++;
		}
		else{
			temper_ = computeTemperature(data[38], data[39]);
			// ROS_INFO_STREAM("Temp is: " << temper);
			tempPacketNum_ = 1;
		}

		if (block < (BLOCKS_PER_PACKET - 1))  // 12
		{
			int azi1, azi2;
			azi1 = 256 * raw->blocks[block + 1].rotation_1 + raw->blocks[block + 1].rotation_2;
			azi2 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
			azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

			// Ingnore the block if the azimuth change abnormal
			if (azimuth_diff <= 0.0 || azimuth_diff > 75.0)
			{
				continue;
			}
		}
		else
		{
			int azi1, azi2;
			azi1 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
			azi2 = 256 * raw->blocks[block - 1].rotation_1 + raw->blocks[block - 1].rotation_2;
			azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

			// Ingnore the block if the azimuth change abnormal
			if (azimuth_diff <= 0.0 || azimuth_diff > 75.0)
			{
				continue;
			}
		}

		for (int firing = 0, k = 0; firing < RS16_FIRINGS_PER_BLOCK; firing++)  // 2
		{
			for (int dsr = 0; dsr < RS16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)  // 16   3
			{
				azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr * RS16_DSR_TOFFSET) + (firing * RS16_FIRING_TOFFSET)) /
					RS16_BLOCK_TDURATION);
				azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;  // convert to integral value...

				union two_bytes tmp;
				tmp.bytes[1] = raw->blocks[block].data[k];
				tmp.bytes[0] = raw->blocks[block].data[k + 1];
				int distance = tmp.uint;

				// read intensity
				intensity = raw->blocks[block].data[k + 2];
				if (Curvesis_new_)
					intensity = calibrateIntensity(intensity, dsr, distance);
				else
					intensity = calibrateIntensity_old(intensity, dsr, distance);

				float distance2 = pixelToDistance(distance, dsr);
				distance2 = distance2 * DISTANCE_RESOLUTION;

				float arg_horiz = (float)azimuth_corrected / 18000.0f * M_PI;
				float arg_vert = VERT_ANGLE_[dsr];
				pcl::PointXYZI point;

				if (distance2 > DISTANCE_MAX || distance2 < DISTANCE_MIN)  // invalid data
				{
					point.x = NAN;
					point.y = NAN;
					point.z = NAN;
					point.intensity = 0;
					pointcloud->at(2 * this->block_num_ + firing, dsr) = point;
				}
				else
				{
					// If you want to fix the rslidar Y aixs to the front side of the cable, please use the two line below
					// point.x = dis * cos(arg_vert) * sin(arg_horiz);
					// point.y = dis * cos(arg_vert) * cos(arg_horiz);

					// If you want to fix the rslidar X aixs to the front side of the cable, please use the two line below
					point.y = -distance2 * cos(arg_vert) * sin(arg_horiz);
					point.x = distance2 * cos(arg_vert) * cos(arg_horiz);
					point.z = distance2 * sin(arg_vert);
					point.intensity = intensity;
					pointcloud->at(2 * this->block_num_ + firing, dsr) = point;
				}
			}
		}
	}

	return;
}


// revision 2019-02-28 by kyungpyo.kim@control-works.co.kr
void rslidar_driver_cw::unpack_RS32(const std::vector<uint8_t>& data, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
{
	float azimuth;  // 0.01 dgree
	float intensity;
	float azimuth_diff;
	float azimuth_corrected_f;
	int azimuth_corrected;

	const raw_packet_t* raw = (const raw_packet_t*)&data[42];

	for (int block = 0; block < BLOCKS_PER_PACKET; block++, this->block_num_++)  // 1 packet:12 data blocks
	{
		if (UPPER_BANK != raw->blocks[block].header)
		{
			std::cout << "skipping RSLIDAR DIFOP packet" << std::endl;
			break;
		}

		if (tempPacketNum_ < 20000 && tempPacketNum_ > 0)  // update temperature information per 20000 packets
		{
			tempPacketNum_++;
		}
		else
		{
			temper_ = computeTemperature(data[38], data[39]);
			// ROS_INFO_STREAM("Temp is: " << temper);
			tempPacketNum_ = 1;
		}

		azimuth = (float)(256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2);

		if (block < (BLOCKS_PER_PACKET - 1))  // 12
		{
			int azi1, azi2;
			azi1 = 256 * raw->blocks[block + 1].rotation_1 + raw->blocks[block + 1].rotation_2;
			azi2 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
			azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

			// Ingnore the block if the azimuth change abnormal
			if (azimuth_diff <= 0.0 || azimuth_diff > 25.0)
			{
				continue;
			}
		}
		else
		{
			int azi1, azi2;
			azi1 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
			azi2 = 256 * raw->blocks[block - 1].rotation_1 + raw->blocks[block - 1].rotation_2;
			azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

			// Ingnore the block if the azimuth change abnormal
			if (azimuth_diff <= 0.0 || azimuth_diff > 25.0)
			{
				continue;
			}
		}

		// Estimate the type of packet
		union two_bytes tmp_flag;
		tmp_flag.bytes[1] = raw->blocks[block].data[0];
		tmp_flag.bytes[0] = raw->blocks[block].data[1];
		int ABflag = isABPacket(tmp_flag.uint);

		int k = 0;
		int index;
		for (int dsr = 0; dsr < RS32_SCANS_PER_FIRING * RS32_FIRINGS_PER_BLOCK; dsr++, k += RAW_SCAN_SIZE)  // 16   3
		{
			if (ABflag == 1 && dsr < 16)
			{
				index = k + 48;
			}
			else if (ABflag == 1 && dsr >= 16)
			{
				index = k - 48;
			}
			else
			{
				index = k;
			}

			int dsr_temp;
			if (dsr >= 16)
			{
				dsr_temp = dsr - 16;
			}
			else
			{
				dsr_temp = dsr;
			}
			azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr_temp * RS32_DSR_TOFFSET)) / RS32_BLOCK_TDURATION);
			azimuth_corrected = correctAzimuth(azimuth_corrected_f, dsr);

			union two_bytes tmp;
			tmp.bytes[1] = raw->blocks[block].data[index];
			tmp.bytes[0] = raw->blocks[block].data[index + 1];
			int ab_flag_in_block = isABPacket(tmp.uint);
			int distance = tmp.uint - ab_flag_in_block * 32768;

			// read intensity
			intensity = (float)raw->blocks[block].data[index + 2];
			if (Curvesis_new_)
				intensity = calibrateIntensity(intensity, dsr, distance);
			else
				intensity = calibrateIntensity_old(intensity, dsr, distance);

			float distance2 = pixelToDistance(distance, dsr);
			distance2 = distance2 * DISTANCE_RESOLUTION;

			float arg_horiz = (float)azimuth_corrected / 18000.0f * M_PI;
			float arg_vert = VERT_ANGLE_[dsr];
			pcl::PointXYZI point;

			if (distance2 > DISTANCE_MAX || distance2 < DISTANCE_MIN)  // invalid data
			{
				point.x = NAN;
				point.y = NAN;
				point.z = NAN;
				point.intensity = 0;
				pointcloud->at(this->block_num_, dsr) = point;
			}
			else
			{
				// If you want to fix the rslidar Y aixs to the front side of the cable, please use the two line below
				// point.x = dis * cos(arg_vert) * sin(arg_horiz);
				// point.y = dis * cos(arg_vert) * cos(arg_horiz);

				// If you want to fix the rslidar X aixs to the front side of the cable, please use the two line below
				point.y = -distance2 * cos(arg_vert) * sin(arg_horiz);
				point.x = distance2 * cos(arg_vert) * cos(arg_horiz);
				point.z = distance2 * sin(arg_vert);
				point.intensity = intensity;
				pointcloud->at(this->block_num_, dsr) = point;
			}
		}
	}
}



// revision 2019-02-28 by kyungpyo.kim@control-works.co.kr
//------------------------------------------------------------
//校准反射强度值 (보정 된 반사 강도 값)
float rslidar_driver_cw::calibrateIntensity(float intensity, int calIdx, int distance)
{
	int algDist;
	int sDist;
	int uplimitDist;
	float realPwr;
	float refPwr;
	float tempInten;
	float distance_f;
	float endOfSection1, endOfSection2;

	int temp = estimateTemperature(temper_);

	realPwr = std::max((float)(intensity / (1 + (temp - TEMPERATURE_MIN) / 24.0f)), 1.0f);
	// realPwr = intensity;

	if (intensity_mode_ == 1)
	{
		// transform the one byte intensity value to two byte
		if ((int)realPwr < 126)
			realPwr = realPwr * 4.0f;
		else if ((int)realPwr >= 126 && (int)realPwr < 226)
			realPwr = (realPwr - 125.0f) * 16.0f + 500.0f;
		else
			realPwr = (realPwr - 225.0f) * 256.0f + 2100.0f;
	}
	else if (intensity_mode_ == 2)
	{
		// the caculation for the firmware after T6R23V8(16) and T9R23V6(32)
		if ((int)realPwr < 64)
			realPwr = realPwr;
		else if ((int)realPwr >= 64 && (int)realPwr < 176)
			realPwr = (realPwr - 64.0f) * 4.0f + 64.0f;
		else
			realPwr = (realPwr - 176.0f) * 16.0f + 512.0f;
	}
	else
	{
		std::cout << "The intensity mode is not right" << std::endl;
	}

	int indexTemper = estimateTemperature(temper_) - TEMPERATURE_MIN;
	uplimitDist = g_ChannelNum_[calIdx][indexTemper] + 20000;
	// limit sDist
	sDist = (distance > g_ChannelNum_[calIdx][indexTemper]) ? distance : g_ChannelNum_[calIdx][indexTemper];
	sDist = (sDist < uplimitDist) ? sDist : uplimitDist;
	// minus the static offset (this data is For the intensity cal useage only)
	algDist = sDist - g_ChannelNum_[calIdx][indexTemper];

	// calculate intensity ref curves
	float refPwr_temp = 0.0f;
	int order = 3;
	endOfSection1 = 500.0f;
	endOfSection2 = 4000.0;
	distance_f = (float)algDist;
	if (intensity_mode_ == 1)
	{
		if (distance_f <= endOfSection1)
		{
			refPwr_temp =
				aIntensityCal_[0][calIdx] * exp(aIntensityCal_[1][calIdx] - aIntensityCal_[2][calIdx] * distance_f / 100.0f) +
				aIntensityCal_[3][calIdx];
			//   printf("a-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
		}
		else
		{
			for (int i = 0; i < order; i++)
			{
				refPwr_temp += aIntensityCal_[i + 4][calIdx] * (pow(distance_f / 100.0f, order - 1 - i));
			}
			// printf("b-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
		}
	}
	else if (intensity_mode_ == 2)
	{
		if (distance_f <= endOfSection1)
		{
			refPwr_temp =
				aIntensityCal_[0][calIdx] * exp(aIntensityCal_[1][calIdx] - aIntensityCal_[2][calIdx] * distance_f / 100.0f) +
				aIntensityCal_[3][calIdx];
			//   printf("a-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
		}
		else if (distance_f > endOfSection1 && distance_f <= endOfSection2)
		{
			for (int i = 0; i < order; i++)
			{
				refPwr_temp += aIntensityCal_[i + 4][calIdx] * (pow(distance_f / 100.0f, order - 1 - i));
			}
			// printf("b-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
		}
		else
		{
			float refPwr_temp0 = 0.0f;
			float refPwr_temp1 = 0.0f;
			for (int i = 0; i < order; i++)
			{
				refPwr_temp0 += aIntensityCal_[i + 4][calIdx] * (pow(4000.0f / 100.0f, order - 1 - i));
				refPwr_temp1 += aIntensityCal_[i + 4][calIdx] * (pow(3900.0f / 100.0f, order - 1 - i));
			}
			refPwr_temp = 0.3f * (refPwr_temp0 - refPwr_temp1) * distance_f / 100.0f + refPwr_temp0;
		}
	}
	else
	{
		std::cout << "The intensity mode is not right" << std::endl;
	}

	refPwr = std::max(std::min(refPwr_temp, 500.0f), 4.0f);

	tempInten = (intensityFactor_ * refPwr) / realPwr;
	if (numOfLasers_ == 32)
	{
		tempInten = tempInten * CurvesRate_[calIdx];
	}
	tempInten = (int)tempInten > 255 ? 255.0f : tempInten;
	return tempInten;
}

//------------------------------------------------------------
//校准反射强度值 (보정 된 반사 강도 값) old
/// revision 2019-02-28 by kyungpyo.kim@control-works.co.kr
float rslidar_driver_cw::calibrateIntensity_old(float intensity, int calIdx, int distance)
{
	int algDist;
	int sDist;
	int uplimitDist;
	float realPwr;
	float refPwr;
	float tempInten;

	int temp = estimateTemperature(temper_);
	realPwr = std::max((float)(intensity / (1 + (temp - TEMPERATURE_MIN) / 24.0f)), 1.0f);
	// realPwr = intensity;

	if ((int)realPwr < 126)
		realPwr = realPwr * 4.0f;
	else if ((int)realPwr >= 126 && (int)realPwr < 226)
		realPwr = (realPwr - 125.0f) * 16.0f + 500.0f;
	else
		realPwr = (realPwr - 225.0f) * 256.0f + 2100.0f;

	int indexTemper = estimateTemperature(temper_) - TEMPERATURE_MIN;
	uplimitDist = g_ChannelNum_[calIdx][indexTemper] + 1400;
	sDist = (distance > g_ChannelNum_[calIdx][indexTemper]) ? distance : g_ChannelNum_[calIdx][indexTemper];
	sDist = (sDist < uplimitDist) ? sDist : uplimitDist;
	// minus the static offset (this data is For the intensity cal useage only)
	algDist = sDist - g_ChannelNum_[calIdx][indexTemper];
	// algDist = algDist < 1400? algDist : 1399;
	refPwr = aIntensityCal_old_[algDist][calIdx];

	tempInten = (51 * refPwr) / realPwr;
	if (numOfLasers_ == 32)
	{
		tempInten = tempInten * CurvesRate_[calIdx];
	}
	tempInten = (int)tempInten > 255 ? 255.0f : tempInten;
	return tempInten;
}

//------------------------------------------------------------
float rslidar_driver_cw::pixelToDistance(int pixelValue, int passageway)
{
	float DistanceValue;
	int indexTemper = estimateTemperature(temper_) - TEMPERATURE_MIN;
	if (pixelValue <= g_ChannelNum_[passageway][indexTemper])
	{
		DistanceValue = 0.0;
	}
	else
	{
		DistanceValue = (float)(pixelValue - g_ChannelNum_[passageway][indexTemper]);
	}
	return DistanceValue;
}

//------------------------------------------------------------
int rslidar_driver_cw::estimateTemperature(float Temper)
{
	int temp = (int)floor(Temper + 0.5);
	if (temp < TEMPERATURE_MIN)
	{
		temp = TEMPERATURE_MIN;
	}
	else if (temp > TEMPERATURE_MIN + TEMPERATURE_RANGE_)
	{
		temp = TEMPERATURE_MIN + TEMPERATURE_RANGE_;
	}

	return temp;
}

//------------------------------------------------------------
float rslidar_driver_cw::computeTemperature(unsigned char bit1, unsigned char bit2)
{
	float Temp;
	float bitneg = bit2 & 128;   // 10000000
	float highbit = bit2 & 127;  // 01111111
	float lowbit = bit1 >> 3;
	if (bitneg == 128)
	{
		Temp = -1 * (highbit * 32 + lowbit) * 0.0625f;
	}
	else
	{
		Temp = (highbit * 32 + lowbit) * 0.0625f;
	}

	return Temp;
}

//------------------------------------------------------------
int rslidar_driver_cw::isABPacket(int distance)
{
	int ABflag = 0;
	if ((distance & 32768) != 0)
	{
		ABflag = 1;  // B
	}
	else
	{
		ABflag = 0;  // A
	}
	return ABflag;
}

//------------------------------------------------------------
int rslidar_driver_cw::correctAzimuth(float azimuth_f, int passageway)
{
	int azimuth;
	if (azimuth_f > 0.0 && azimuth_f < 3000.0)
	{
		azimuth_f = azimuth_f + HORI_ANGLE_[passageway] + 36000.0f;
	}
	else
	{
		azimuth_f = azimuth_f + HORI_ANGLE_[passageway];
	}
	azimuth = (int)azimuth_f;
	azimuth %= 36000;

	return azimuth;
}
