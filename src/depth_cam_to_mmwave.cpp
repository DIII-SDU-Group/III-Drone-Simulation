#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cstdlib>
#include <stdlib.h> 
#include <iostream>
#include <chrono>
#include <ctime>    
#include <math.h>  
#include <limits>
#include <vector>
#include <string>
#include <stdint.h>
#include <cstdlib> 
#include <ctime> 
#include <random>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "iii_drone_core/utils/math.hpp"
#include "iii_drone_configuration/configurator.hpp"

using namespace iii_drone::math;

using namespace std::chrono_literals;

class DepthCamToMmwave : public rclcpp::Node
{

	public:
		DepthCamToMmwave() : Node("depth_cam_to_mmwave", "/depth_cam_to_mmwave"),
        configurator_(this,"depth_cam_to_mmwave") {

      depth_cam_frame_id_ = configurator_.GetParameter("depth_cam_frame_id").as_string();
      mmwave_frame_id_ = configurator_.GetParameter("mmwave_frame_id").as_string();

			depth_cam_to_mmwave_pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor/mmwave/pcl", 10);
			filtered_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);
			received_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("received_points", 10);
			clustered_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clustered_points", 10);

			subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/depth_camera/points",	10,
			std::bind(&DepthCamToMmwave::depth_cam_to_mmwave_pcl, this, std::placeholders::_1));
		}

		~DepthCamToMmwave() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down depth_cam_to_mmwave_converter..");
		}


	private:
    iii_drone::configuration::Configurator<rclcpp::Node> configurator_;
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cam_to_mmwave_pcl_publisher_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr received_points_publisher_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_publisher_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_points_publisher_;

		std::vector<float> objects_dist;
		std::vector<float> objects_angl;
		void depth_cam_to_mmwave_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr _msg);
		pcl::PointCloud<pcl::PointXYZ>::Ptr eucClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void publishPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, std::string frame_id);
    std::string depth_cam_frame_id_;
    std::string mmwave_frame_id_;
};



void DepthCamToMmwave::depth_cam_to_mmwave_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

  std::srand(static_cast<unsigned int>(this->get_clock()->now().nanoseconds()));

  unsigned int pcl_size = msg->width;

  RCLCPP_DEBUG(this->get_logger(), "Received %d points in msg", pcl_size);
  uint8_t *ptr = msg->data.data();
  const uint32_t POINT_STEP = 24;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*msg, *cloud);
	
  // for (size_t i = 0; i < pcl_size; i++) {

  // pcl::PointXYZ point(
  //         (float)(*(reinterpret_cast<float*>(ptr + 0))),
  //         (float)(*(reinterpret_cast<float*>(ptr + 4))),
  //         (float)(*(reinterpret_cast<float*>(ptr + 8)))
  //     );

  // cloud->push_back(point);

      
  //     ptr += POINT_STEP;

  // }   

  // cloud->width = pcl_size;
  // cloud->height = 1;

	eucClustering(cloud);


}


pcl::PointCloud<pcl::PointXYZ>::Ptr DepthCamToMmwave::eucClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  RCLCPP_DEBUG(get_logger(), "PointCloud before filtering has: %d data points.", cloud->size());

  publishPoints(cloud, received_points_publisher_, depth_cam_frame_id_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  for (unsigned int i = 0; i < cloud->size(); i++) {

    if (!pcl::isFinite(cloud->at(i))) {
      continue;
    }

    // if ((float) rand()/RAND_MAX > 0.7) {

      cloud_f->push_back((*cloud)[i]);

    // }
  }

  // Drop points more than 20 m away
  for (unsigned int i = 0; i < cloud_f->size(); i++) {

    float dist = sqrt(cloud_f->at(i).x*cloud_f->at(i).x + cloud_f->at(i).y*cloud_f->at(i).y + cloud_f->at(i).z*cloud_f->at(i).z);

    // RCLCPP_DEBUG(get_logger(), "Distance: %f", dist);

    if (dist < 10)
      cloud_filtered->push_back(cloud_f->at(i));

  }

  RCLCPP_DEBUG(get_logger(), "PointCloud after filtering has: %d data points.", cloud_filtered->size());

  publishPoints(cloud_filtered, filtered_points_publisher_, depth_cam_frame_id_);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (1.2); 
  ec.setMinClusterSize (5);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pl_points (new pcl::PointCloud<pcl::PointXYZ>);

  int j = 0;
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*cloud_filtered)[idx]);
    } //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    float x = 0;
    float y = 0; 
    float z = 0;

    for (unsigned int i = 0; i < cloud_cluster->size(); i++) {

      pcl::PointXYZ point = cloud_cluster->at(i);

      x += point.x;
      y += point.y;
      z += point.z;

    }

    x /= cloud_cluster->size();
    y /= cloud_cluster->size();
    z /= cloud_cluster->size();

    pcl::PointXYZ pl_point (x, y, z);
    pl_points->push_back(pl_point);

    RCLCPP_DEBUG(get_logger(), "PointCloud representing the Cluster: %d data points.", cloud_cluster->size());
    j++;
  }

  publishPoints(pl_points, clustered_points_publisher_, depth_cam_frame_id_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pl_noise_points (new pcl::PointCloud<pcl::PointXYZ>);


  for (unsigned int i = 0; i < pl_points->size(); i++) {

    pcl::PointXYZ point = pl_points->at(i);

    float dist = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);

    float factor = dist * 0.05;

    float noise = (float) rand()/RAND_MAX;
    noise -= 0.5;
    point.x += noise*factor;

    noise = (float) rand()/RAND_MAX;
    noise -= 0.5;
    point.y += noise*factor;

    noise = (float) rand()/RAND_MAX;
    noise -= 0.5;
    point.z += noise*factor;

    pcl::PointXYZ new_point(point.x, -point.y, -point.z);

    pl_noise_points->push_back(new_point);

  }

  publishPoints(pl_noise_points, depth_cam_to_mmwave_pcl_publisher_, mmwave_frame_id_);

  return cloud;
}


void DepthCamToMmwave::publishPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, std::string frame_id) {

    auto pcl2_msg = sensor_msgs::msg::PointCloud2();
    pcl2_msg.header.frame_id = frame_id;
    pcl2_msg.header.stamp = this->get_clock()->now();

    pcl2_msg.fields.resize(3);
    pcl2_msg.fields[0].name = 'x';
    pcl2_msg.fields[0].offset = 0;
    pcl2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg.fields[0].count = 1;
    pcl2_msg.fields[1].name = 'y';
    pcl2_msg.fields[1].offset = 4;
    pcl2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg.fields[1].count = 1;
    pcl2_msg.fields[2].name = 'z';
    pcl2_msg.fields[2].offset = 8;
    pcl2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg.fields[2].count = 1;

    const uint32_t POINT_STEP = 12;

    if(cloud->size() > 0){

        pcl2_msg.data.resize(std::max((size_t)1, cloud->size()) * POINT_STEP, 0x00);

    } else {

        return;

    }

    pcl2_msg.point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
    pcl2_msg.row_step = pcl2_msg.data.size();//pcl2_msg.point_step * pcl2_msg.width; // only 1 row because unordered
    pcl2_msg.height = 1; // because unordered cloud
    pcl2_msg.width = pcl2_msg.row_step / POINT_STEP; // number of points in cloud
    pcl2_msg.is_dense = false; // there may be invalid points

    uint8_t *pcl2_ptr = pcl2_msg.data.data();

    for (unsigned int i = 0; i < cloud->size(); i++) {
        pcl::PointXYZ point = (*cloud)[i];

        *(reinterpret_cast<float*>(pcl2_ptr + 0)) = point.x;
        *(reinterpret_cast<float*>(pcl2_ptr + 4)) = point.y;
        *(reinterpret_cast<float*>(pcl2_ptr + 8)) = point.z;
        pcl2_ptr += POINT_STEP;

    }

    pub->publish(pcl2_msg);

}

	
			
int main(int argc, char *argv[])
{
	std::cout << "Starting depth_cam_to_mmwave_converter node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DepthCamToMmwave>());

	rclcpp::shutdown();
	return 0;
}
