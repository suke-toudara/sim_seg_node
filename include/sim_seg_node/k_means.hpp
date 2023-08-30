#ifndef K_Means_Node_
#define K_Means_Node_


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/opencv.hpp>
#include "segmentation_msg/msg/segmentation_info.hpp"

class K_Means_Node : public rclcpp::Node
{
public:
  K_Means_Node() ;
  ~K_Means_Node();

private:
  int class_num ;
  std::string mode ;
  //cv2 
  cv_bridge::CvImagePtr cv_image_ptr;
  //Publisher
  rclcpp::Publisher<segmentation_msg::msg::SegmentationInfo>::SharedPtr k_means_pub_;
  //Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  //callback
  void image_callback(const sensor_msgs::msg::Image & image_msg) ;
};
#endif
