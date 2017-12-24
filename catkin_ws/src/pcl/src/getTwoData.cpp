#include <ros/ros.h>
// PCL specific includes

#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include "pcl_ros/point_cloud.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
ros::Publisher point_cloud_combine;
ros::Publisher pointRGB_A;
ros::Publisher pointRGB_B;
PointCloudRGB::Ptr cloud_A(new PointCloudRGB);
PointCloudRGB::Ptr cloud_B(new PointCloudRGB);

void combineTwoPCL(PointCloudRGB::Ptr&, PointCloudRGB::Ptr&);
void cloud_cb_A(const sensor_msgs::PointCloud2ConstPtr& input){
	pcl::fromROSMsg(*input, *cloud_A);// convert from PCL2 to pcl point type
	printf("cloud A size: %d\n", cloud_A->points.size());
	for(int i=0; i<cloud_A->points.size(); i++){
		cloud_A->points[i].r = 255;
		cloud_A->points[i].g = 255;
		cloud_A->points[i].b = 255;
	}
	//pointRGB_A.publish(*cloud_A);	
}

void cloud_cb_B(const sensor_msgs::PointCloud2ConstPtr& input){
	pcl::fromROSMsg(*input, *cloud_B);
	printf("cloud B size: %d\n", cloud_B->points.size());
	combineTwoPCL(cloud_A, cloud_B);
	for(int i=0; i<cloud_B->points.size(); i++){
		cloud_B->points[i].r = 150;
		cloud_B->points[i].g = 0;
		cloud_B->points[i].b = 150;
		cloud_B->points[i].x += 2.5;
		cloud_B->points[i].y += 0.8;
		cloud_B->points[i].z += 2.0;
	}
	//pointRGB_B.publish(*cloud_B);
}

void combineTwoPCL(PointCloudRGB::Ptr& first_cloud, PointCloudRGB::Ptr& second_cloud){
	// combine two pcl set
	PointCloudRGB::Ptr combine_cloud(new PointCloudRGB);
	sensor_msgs::PointCloud2 combine_ros_point_cloud2;
	
	printf("first cloud size: %d\n", first_cloud->points.size());
	printf("second cloud size: %d\n", second_cloud->points.size());

	first_cloud->header.frame_id = "/camera_link";	
	second_cloud->header.frame_id = "/camera_link";	
	pointRGB_A.publish(*first_cloud);	
	pointRGB_B.publish(*second_cloud);
	combine_cloud->header.frame_id = "/camera_link";	
	combine_cloud->width = first_cloud->width * 2;
	combine_cloud->height = first_cloud->height;
	//combine_cloud.isdense = first_cloud->isdense;
	combine_cloud->points.resize(combine_cloud->width * combine_cloud->height);
	for(int i=0;i<first_cloud->points.size();i++){
		combine_cloud->points[i] = first_cloud->points[i];
		combine_cloud->points[i+first_cloud->points.size()] = second_cloud->points[i];
	}
	pcl::toROSMsg(*combine_cloud, combine_ros_point_cloud2);
	point_cloud_combine.publish(combine_ros_point_cloud2);
	printf("combine pcl size : %d\n", combine_cloud->points.size());
}


int main(int argc, char** argv){
	ros::init(argc, argv, "combine_pcl");
	ros::NodeHandle nh;
	point_cloud_combine = nh.advertise<sensor_msgs::PointCloud2>("combine_point_cloud", 1);
	pointRGB_A = nh.advertise<PointCloudRGB>("A_RGB", 1);
	pointRGB_B = nh.advertise<PointCloudRGB>("B_RGB", 1);
	ros::Subscriber point_A = nh.subscribe<sensor_msgs::PointCloud2>("/abc/depth/points", 1, cloud_cb_A);
	ros::Subscriber point_B = nh.subscribe<sensor_msgs::PointCloud2>("/def/depth/points", 1, cloud_cb_B);
	//combineTwoPCL(cloud_A, cloud_B);
	//point_cloud_combine = nh.advertise<PointCloudRGB>("/combine/points", 1);
	ros::spin();
}

