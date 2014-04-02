/*
 * PointCloudToSurface.cpp
 *
 *  Created on: Mar 17, 2014
 *      Author: kent
 */

#include "PointCloudToSurface.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "PointCloudToSurface");

	PointCloudToSurface pctsNode;
	pctsNode.makeItSpin();

	return 0;
}

PointCloudToSurface::PointCloudToSurface() : 	viewer(new pcl::visualization::PCLVisualizer("3D Visualizer")),
						cloud(new pcl::PointCloud<PointT>())
{
	//	ROS node handler stuff
	this->nodeHandler = ros::NodeHandle("~");
	this->nodeHandler.param<int>("loop_rate", this->loopRate, 20);

	//	Setup system input
	this->nodeHandler.param<std::string>("cloud_topic", this->input.cloudTopic, "/camera/depth/points");
	this->input.subscriber = this->nodeHandler.subscribe<sensor_msgs::PointCloud2>(this->input.cloudTopic, 10, &PointCloudToSurface::pointCloudCallback, this);

	//	Visualizer
	this->viewer->registerKeyboardCallback(&PointCloudToSurface::keyboardCallback, *this, (void*)&viewer);
	this->viewer->addCoordinateSystem(1.0,0);
	this->viewer->initCameraParameters();
}

PointCloudToSurface::~PointCloudToSurface() {
	// TODO Auto-generated destructor stub
}

void PointCloudToSurface::makeItSpin()
{
	ros::Rate r(this->loopRate);
	bool stop = false;

	while ((!this->viewer->wasStopped()) && ros::ok() && this->nodeHandler.ok())
	{
		//	Update event queue
		ros::spinOnce();

		//	Do stuff
		if (this->cloud->size())
		{
//			printf("\n Reading: \n");
//			printf(" ================================= \n");
//			printf(" Height: %d         Width: %d\n", this->cloud->height, this->cloud->width);
//			printf(" IsDense: %d  IsOrganized: %d\n", this->cloud->is_dense, this->cloud->isOrganized());
//			printf(" Number of points: %d\n", this->cloud->size());

			//this->viewer->removePointCloud("cloud", 0);
			//this->viewer->addPointCloud<PointT>(this->cloud, "cloud", 0);
			//this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

			if (this->firstPointCloud && !stop)
			{
				stop = true;

				//	Do surface stuff
				// Normal estimation*
				pcl::NormalEstimation<PointT, pcl::Normal> n;
				pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
				pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
				tree->setInputCloud(this->cloud);
				n.setInputCloud(this->cloud);
				n.setSearchMethod(tree);
				n.setKSearch(20);
				n.compute(*normals);
				//* normals should not contain the point normals + surface curvatures

				// Concatenate the XYZ and normal fields*
				pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
				pcl::concatenateFields (*this->cloud, *normals, *cloud_with_normals);
				//* cloud_with_normals = cloud + normals

				// Create search tree*
				pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
				tree2->setInputCloud (cloud_with_normals);

				// Initialize objects
				pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
				pcl::PolygonMesh triangles;

				// Set the maximum distance between connected points (maximum edge length)
				gp3.setSearchRadius (0.025);

				// Set typical values for the parameters
				gp3.setMu (2.5);
				gp3.setMaximumNearestNeighbors (100);
				gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
				gp3.setMinimumAngle(M_PI/18); // 10 degrees
				gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
				gp3.setNormalConsistency(false);

				// Get result
				gp3.setInputCloud (cloud_with_normals);
				gp3.setSearchMethod (tree2);
				gp3.reconstruct (triangles);

				// Additional vertex information
				std::vector<int> parts = gp3.getPartIDs();
				std::vector<int> states = gp3.getPointStates();

				//	Push triangles to visualizer
				this->viewer->addPolygonMesh(triangles, "mesh", 0);
			}
		}

		//	Update viewport
		this->viewer->spinOnce(1, true);

		//	Delay looping according to rate
		r.sleep();
	}

	printf("\n Closing visualizer and ROS node .... \n\n");
}

void PointCloudToSurface::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
	pcl::fromROSMsg(*data, *this->cloud);
	this->firstPointCloud = true;
}

void PointCloudToSurface::keyboardCallback(const pcl::visualization::KeyboardEvent &event, void* userdata)
{
	ROS_INFO(" Keyboard callback!!! ");
}
