#include "nodehandler.h"


using namespace std;
using namespace message_filters;
using namespace Eigen;

namespace map_recon {

DynaMapNodeHandler::DynaMapNodeHandler(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
  
  ROS_ERROR("node is runnig....");
  if (!ReadConfig_map() && !MapManager.ReadConfig_run(nodeHandle)){
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  MapManager.ReadConfig_run(nodeHandle);

  DynaMapMsg.frame_num = 0;
  DynaMapMsg.iteration_num = 0;
  DynaMapMsg.t_error = 0.0;
  DynaMapMsg.rmse = 0.0;

  subscriber_depth.subscribe( nodeHandle , depth_image_SubscriberTopic , 500 );
  subscriber_rgb.subscribe( nodeHandle , rgb_image_SubscriberTopic , 500 );

  pcl_Publisher_ = nodeHandle_.advertise<pcl::PointCloud<pcl::PointXYZRGB>> (pcl_PublisherTopic, 1);
  map_Publisher_ = nodeHandle_.advertise<pcl::PointCloud<pcl::PointXYZRGB>> (map_PublisherTopic, 1);
  depth_image_Publisher_ = nodeHandle_.advertise<sensor_msgs::Image>(depthImage_PublisherTopic, 1);

  marker_pub = nodeHandle_.advertise<visualization_msgs::Marker>("Vmarker", 10);
  Log_pub = nodeHandle_.advertise<map_recon::DynaMapMsg>("DynaMapMsg", 1);

  // ExactTime or ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
  sync_.reset(new sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth ));
  sync_->registerCallback(boost::bind(&map_recon::DynaMapNodeHandler::kinect_callback, this, _1, _2));
  ROS_INFO("Successfully launched node.");
}

DynaMapNodeHandler::~DynaMapNodeHandler() {
}

bool DynaMapNodeHandler::ReadConfig_map() {
  if (!nodeHandle_.getParam("rgb_image_SubscriberTopic",
                            rgb_image_SubscriberTopic) ||
      !nodeHandle_.getParam("depth_image_SubscriberTopic",
                            depth_image_SubscriberTopic) ||
      !nodeHandle_.getParam("pcl_PublisherTopic",
                            pcl_PublisherTopic) ||
      !nodeHandle_.getParam("map_PublisherTopic",
                            map_PublisherTopic) ||
      !nodeHandle_.getParam("depthImage_PublisherTopic",
                              depthImage_PublisherTopic))
    return false;
  return true;
}

void DynaMapNodeHandler::rgb_img_callback(const sensor_msgs::Image::ConstPtr& input_img) {
  ROS_INFO("I received rgb image with height: [%i]", 
    input_img->height);
}

void DynaMapNodeHandler::depth_img_callback(const sensor_msgs::Image::ConstPtr& input_img){
   ROS_INFO("I received depth image with height: [%i]", 
    input_img->height);
}

void DynaMapNodeHandler::Evaluatation(string frame_id, string reference_frame_id, Matrix4f& source_tf){
  Matrix4f gt_tf = lookupTransformToEigen(frame_id, reference_frame_id ,_transform);

  Matrix3f gt_Rotation;
  gt_Rotation << gt_tf(0,0),gt_tf(0,1),gt_tf(0,2),
                 gt_tf(1,0),gt_tf(1,1),gt_tf(1,2),
                 gt_tf(2,0),gt_tf(2,1),gt_tf(2,2);
  Vector3f gt_Translation (gt_tf(0,3),gt_tf(1,3),gt_tf(2,3));

  Matrix3f s_Rotation;
  s_Rotation << source_tf(0,0),source_tf(0,1),source_tf(0,2),
                source_tf(1,0),source_tf(1,1),source_tf(1,2),
                source_tf(2,0),source_tf(2,1),source_tf(2,2);
  Vector3f s_Translation (source_tf(0,3),source_tf(1,3),source_tf(2,3)); 

  translation_error += gt_Translation - s_Translation;
  float distant_e = sqrt(pow((gt_tf(0,3)-source_tf(0,3)),2)+pow((source_tf(1,3)-gt_tf(1,3)),2)+pow((gt_tf(2,3)-source_tf(2,3)),2));
  t_errors.push_back(distant_e);
  DynaMapMsg.t_error += distant_e;
  DynaMapMsg.avg_t_error = accumulate( t_errors.begin(), t_errors.end(), 0.0)/t_errors.size();
  
  Rotation_error = (s_Rotation*gt_Rotation.transpose()) * Rotation_error.transpose();

  d_errors.push_back(pow(distant_e,2));
  DynaMapMsg.rmse = sqrt(std::accumulate(d_errors.begin(), d_errors.end(), 0.0)/DynaMapMsg.frame_num);

  DynaMapMsg.avg_runtime = accumulate( runtimes.begin(), runtimes.end(), 0.0)/runtimes.size();
  Log_pub.publish(DynaMapMsg);
}

Matrix4f DynaMapNodeHandler::lookupTransformToEigen(string frame_id, string reference_frame_id ,tf::StampedTransform _transform){
    Matrix4f _transformation;
    for(int cnt=0; cnt<5; cnt++){
        try{
          _listener.lookupTransform(frame_id,reference_frame_id,   
                                    ros::Time(0), _transform);
          break;
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(3.0).sleep();
        }
      }

      Eigen::Affine3d transform_eigen;
      TransformTFToEigenImpl(_transform,transform_eigen);
      _transformation(0,0) = static_cast<float>(transform_eigen.rotation()(0,0));
      _transformation(1,0) = static_cast<float>(transform_eigen.rotation()(1,0));
      _transformation(2,0) = static_cast<float>(transform_eigen.rotation()(2,0));
      _transformation(3,0) = 0;

      _transformation(0,1) = static_cast<float>(transform_eigen.rotation()(0,1));
      _transformation(1,1) = static_cast<float>(transform_eigen.rotation()(1,1));
      _transformation(2,1) = static_cast<float>(transform_eigen.rotation()(2,1));
      _transformation(3,1) = 0;

      _transformation(0,2) = static_cast<float>(transform_eigen.rotation()(0,2));
      _transformation(1,2) = static_cast<float>(transform_eigen.rotation()(1,2));
      _transformation(2,2) = static_cast<float>(transform_eigen.rotation()(2,2));
      _transformation(3,2) = 0;

      _transformation(0,3) = static_cast<float>(transform_eigen.translation().x());
      _transformation(1,3) = static_cast<float>(transform_eigen.translation().y());
      _transformation(2,3) = static_cast<float>(transform_eigen.translation().z());
      _transformation(3,3) = 1;

    return _transformation;
}

void DynaMapNodeHandler::tf_publisher(string frame_id, string child_frame_id, ros::Time time_stamp, Eigen::Matrix4f transformation){
  // Matrix4f Trans;
  // Trans <<  0, 0,-1, 0,
  //           0, 1, 0, 0,
  //           1, 0, 0, 0,
  //           0, 0, 0, 1;
  // transformation = Trans * transformation;
  tf::Vector3 origin;
  origin.setValue(static_cast<double>(transformation(0,3)),static_cast<double>(transformation(1,3)),static_cast<double>(transformation(2,3)));
  
  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(transformation(0,0)), static_cast<double>(transformation(0,1)), static_cast<double>(transformation(0,2)), 
        static_cast<double>(transformation(1,0)), static_cast<double>(transformation(1,1)), static_cast<double>(transformation(1,2)), 
        static_cast<double>(transformation(2,0)), static_cast<double>(transformation(2,1)), static_cast<double>(transformation(2,2)));

  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);

  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(tfqt);
  _broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
}

void DynaMapNodeHandler::TransformTFToEigenImpl(const tf::Transform &t, Eigen::Affine3d &e){
  for(int i=0; i<3; i++){
    e.matrix()(i,3) = t.getOrigin()[i];
    for(int j=0; j<3; j++){
      e.matrix()(i,j) = t.getBasis()[i][j];
    }
  }
  // Fill in identity in last row
  for (int col = 0 ; col < 3; col ++)
    e.matrix()(3, col) = 0;
  e.matrix()(3,3) = 1;
}

void DynaMapNodeHandler::kinect_callback(const sensor_msgs::ImageConstPtr& msg_depth, const sensor_msgs::ImageConstPtr& msg_rgb) {
  clock_t begin = clock();
  MapManager.RegOK = false;
  MapManager.FirtIteration = true;
  double last_error = 0.0;
  double error = 0.0;
  Matrix4f ResultTrans;
  ResultTrans = Matrix4f::Identity();

  cv_bridge::CvImagePtr img_ptr_rgb;
  cv_bridge::CvImagePtr img_ptr_depth;


  try{
    img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }
  try{
    img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb,sensor_msgs::image_encodings::TYPE_8UC3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  cv::Mat new_rgb_img(img_ptr_rgb->image.rows, img_ptr_rgb->image.cols, CV_8UC3);
  new_rgb_img = img_ptr_rgb->image;
  cv::Mat new_depth_img(img_ptr_depth->image.rows, img_ptr_depth->image.cols, CV_32FC1);
  new_depth_img = img_ptr_depth->image;

  if(DynaMapMsg.frame_num < MapManager.frame_num ){  

    DynaMapMsg.frame_num++;

    if(MapManager.RegOK){

        // estimates the passed time from the moment which raw images are 
        // received to this point which registration of the frame is done.
        clock_t end = clock();
        DynaMapMsg.runtime = float(end - begin) / CLOCKS_PER_SEC;
        runtimes.push_back(DynaMapMsg.runtime);


        if(MapManager.CorPointSamMode == 2 || MapManager.CorPointSamMode == 1){
          MapManager.prev_rgb_img = new_rgb_img;
          MapManager.prev_depth_img = new_depth_img;
          MapManager.prev_descriptors = MapManager.new_descriptors;
          MapManager.prev_keypoints = MapManager.new_keypoints;
          //MapManager.prev_prod = MapManager.new_prod;
        }

        MapManager.clone_pcl(MapManager.prev_cloud, MapManager.curr_cloud);

        // updates and Publishes the TF-tree with estimated frame 
        // of the kinect sensor with name /sensor
        if(MapManager.Publish_TF){
          tf_publisher(MapManager.frame_id, MapManager.child_frame_id,
           ros::Time::now() , MapManager.prevResultTrans);
        }

        // estimates the lines between the corresponding point in both 
        // point-clouds and publishes them to be visulized in rviz
        if(MapManager.Publish_line){ 
          MapManager.Buffer_Lines(MapManager.line_list);
          marker_pub.publish(MapManager.line_list);
          MapManager.line_list.points.clear();
        }
        
        MapManager.RegOK = false;

        // computes and publishes the evaluation parameters
        if(MapManager.En_Evaluatation)Evaluatation(MapManager.frame_id, MapManager.child_frame_id, MapManager.prevResultTrans);
    }

    // print out the debug and log values in the terminal.
    if(MapManager.DebuggLogShow) 
      cout <<"IN: "<< DynaMapMsg.iteration_num <<
             " TNP: " << MapManager.curr_cloud->points.size() << 
             " FNum: " << DynaMapMsg.frame_num << 
             " RT: " << DynaMapMsg.runtime << 
             " AVG-RT: " << DynaMapMsg.avg_runtime << 
             " TE: " << DynaMapMsg.t_error << endl;
  }
}

}  // namespace map_recon ... 