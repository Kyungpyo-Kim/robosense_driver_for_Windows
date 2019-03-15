#pragma once

// STD
#include <string>

// PCL
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>

//CUSTOM
#include "../driver/rsdriver_cw.h"

class visualization_demo {

public:
	visualization_demo() {};

	~visualization_demo() {};
	
	void run() {

		// std::string anglePath("cfg/configuration_data/angle.csv");
		// std::string curvesPath("cfg/configuration_data/curves.csv");
		// std::string channelPath("cfg/configuration_data/ChannelNum.csv");
		// std::string curvesRatePath("");
		// std::string model("RS16");

		std::string anglePath("cfg32/configuration_data/angle.csv");
		std::string curvesPath("cfg32/configuration_data/curves.csv");
		std::string channelPath("cfg32/configuration_data/ChannelNum.csv");
		std::string curvesRatePath("cfg32/configuration_data/CurveRate.csv");
		std::string model("RS32");

		// 0. initailization
		rslidar_driver_cw rd(model);

		// 1. load configuration
		rd.loadConfigFile(
			anglePath,
			curvesPath,
			channelPath,
			curvesRatePath,
			model);

		// 2. start driver
		if (!rd.start_driver()) return;

		// 3. run driver (thread)
		rd.run_driver();

		// 99. run algorithm using "get_point_cloud" function

		// --------------------------------------------
		// -----Open 3D viewer and add point cloud-----
		// --------------------------------------------
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Point Cloud Viewer"));
		

		viewer->setBackgroundColor(0, 0, 0);
		viewer->addCoordinateSystem(1.0);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_draw(new pcl::PointCloud<pcl::PointXYZRGB>);
		viewer->initCameraParameters();
		viewer->addPointCloud(pc_draw, "rslidar point cloud");
		viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "rslidar point cloud");
		
		while (!viewer->wasStopped()) {
			
			viewer->spinOnce();

			// --------------------------------------------
			// --------------------------------------------
			// 99. Get point cloud data from driver
			// --------------------------------------------
			pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
			while (!rd.get_point_cloud(outPoints));;
			// --------------------------------------------

			pc_draw->clear();

			if (!viewer->updatePointCloud(pc_draw, "rslidar point cloud")) {
				viewer->addPointCloud(pc_draw, "rslidar point cloud");
			}

			for (int i = 0; i < outPoints->size(); i+=2) {

				auto it = outPoints->begin() + i;

				pcl::PointXYZRGB p = pcl::PointXYZRGB((int)(255. * it->intensity), 0, 0);
				p.x = it->x; p.y = it->y; p.z = it->z;
				float intensity = it->intensity;
				
				pc_draw->push_back(p);
			}

			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pc_draw);
			if (!viewer->updatePointCloud(pc_draw, rgb, "rslidar point cloud")) {
				viewer->addPointCloud(pc_draw, rgb, "rslidar point cloud");
			}
		}

		// Be careful!
		// wait_driver() has to be located at the end of the algorithm(or main function)
		// refer the join() of the STD Thread
		rd.wait_driver();
	};
};