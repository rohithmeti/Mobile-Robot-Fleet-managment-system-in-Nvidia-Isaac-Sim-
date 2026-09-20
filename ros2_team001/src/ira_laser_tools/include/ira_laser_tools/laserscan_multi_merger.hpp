#include <string.h>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <laser_geometry/laser_geometry.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "pcl_ros/transforms.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std;
using namespace pcl;

namespace ira_laser_tools{
class LaserscanMerger : public rclcpp::Node
{
public:
	LaserscanMerger(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
	void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, std::string topic);
	void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);
	rcl_interfaces::msg::SetParametersResult reconfigureCallback(const std::vector<rclcpp::Parameter> &parameters);

private:
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
	laser_geometry::LaserProjection projector_;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;
	OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
	std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subscribers;
	std::vector<bool> clouds_modified;

	std::vector<pcl::PCLPointCloud2> clouds;
	std::vector<string> input_topics;

	void laserscan_topic_parser();

	double angle_min;
	double angle_max;
	double angle_increment;
	double time_increment;
	double scan_time;
	double range_min;
	double range_max;

	string destination_frame;
	string cloud_destination_topic;
	string scan_destination_topic;
	string laserscan_topics;
};
}