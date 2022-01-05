#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <time.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <message_filters/subscriber.h> // message_filters
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>  // OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/nonfree/nonfree.hpp>

#include <eigen3/Eigen/Dense>
#include "Eigen/Core"
#include "Eigen/Geometry"

#include <algorithm>
#include <numeric>
#include <ctime>
#include <sstream>

#include "include/dynaMap/dynaMap.h"

//#define EXACT
#define APPROXIMATE


#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;


namespace DynaMap {

class DynaMapNodeHandler {
 public:

  DynaMap MapManager;
  int it_num;
  bool FirstFrame;

  std::vector<float> runtimes;
  
  // class Constructor
  DynaMapNodeHandler(ros::NodeHandle& node_handler);
  
  // class Destructor
  virtual ~DynaMapNodeHandler();

  void rgb_img_callback(const sensor_msgs::Image::ConstPtr& input_img);
  void depth_img_callback(const sensor_msgs::Image::ConstPtr& input_img);
  void kinect_callback(const sensor_msgs::ImageConstPtr& msg_depth, const sensor_msgs::ImageConstPtr& msg_rgb);

  void Evaluatation(string frame_id, string reference_frame_id, Matrix4f& source_tf);
  Eigen::Matrix4f lookupTransformToEigen(string frame_idm, string reference_frame_id ,tf::StampedTransform _transform);
  void tf_publisher(string frame_id, string child_frame_id, ros::Time time_stamp, Eigen::Matrix4f transformation);

  void TransformTFToEigenImpl(const tf::Transform &t, Eigen::Affine3d &e);
 private:
  bool ReadConfig_map();
  bool ReadConfig_run(DynaMap& Map_manager);

  // ROS node handle.
  ros::NodeHandle nodeHandle_;

  message_filters::Subscriber<sensor_msgs::Image> subscriber_depth;
  message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb;

  #ifdef EXACT
    typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  #endif
  #ifdef APPROXIMATE
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  #endif
  typedef Synchronizer<MySyncPolicy> sync;
  boost::shared_ptr<sync> sync_;

  ros::Publisher pcl_Publisher_;
  ros::Publisher map_Publisher_;
  ros::Publisher depth_image_Publisher_;
  ros::Publisher marker_pub;

  ros::Publisher Log_pub;

  std::string rgb_image_SubscriberTopic;
  std::string depth_image_SubscriberTopic;
  std::string tf_SubscriberTopic;

  std::string pcl_PublisherTopic;
  std::string map_PublisherTopic;
  std::string depthImage_PublisherTopic;

  tf::TransformBroadcaster _broadcaster;
  tf::StampedTransform _transform;
  tf::TransformListener _listener;

  Matrix3f Rotation_error;
  Vector3f translation_error;
  std::vector<float> d_errors;
  std::vector<float> t_errors;

  // DynaMap::dynaMapMsg dynaMapMsg;


};
}  // namespace DynaMap ...

