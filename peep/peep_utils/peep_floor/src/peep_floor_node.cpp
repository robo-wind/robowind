#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>


void calibrationFunc(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tag_array_msg)
{
  size_t num_detections = tag_array_msg->detections.size();
  for (size_t i = 0; i < num_detections; ++i)
  {
    Eigen::Affine3d transform;
    tf::poseMsgToEigen (tag_array_msg->detections[i].pose.pose, transform);
    transform = transform.inverse();
    
    std::string frame_id = tag_array_msg->detections[i].pose.header.frame_id;
    //Print transform.
    Eigen::Vector3d translation = transform.translation();
    Eigen::Quaterniond quaternion (transform.rotation());
    //std::cout <<"<node pkg=\"tf\" type=\"static_transform_publisher\" name=\""<<frame_id<<"_tag_"<<tag_array_msg->detections[i].id<<"_transformer\" args=\""
    //    <<translation.x()<<" "<<translation.y()<<" "<<translation.z()<<" "
    //    <<quaternion.x()<<" "<<quaternion.y()<<" "<<quaternion.z()<<" "<<quaternion.w()
    //    <<" tag_"<<tag_array_msg->detections[i].id<<" "<<frame_id<<" 40\"/>"<<std::endl;

    std::cout <<"<node pkg=\"tf\" type=\"static_transform_publisher\" name=\""<<frame_id<<"_map_transformer\" args=\""
        <<translation.x()<<" "<<translation.y()<<" "<<translation.z()<<" "
        <<quaternion.x()<<" "<<quaternion.y()<<" "<<quaternion.z()<<" "<<quaternion.w()
        <<" map "<<frame_id<<" 1\"/>"<<std::endl;
    
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "peep_floor_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Subscriber sub_tag_pose = nh.subscribe<apriltags_ros::AprilTagDetectionArray>("tag_detections",5,calibrationFunc);
  ROS_INFO_STREAM ("Subscribed to topic "<<sub_tag_pose.getTopic());
  
  // attributes::CaffeClassifierRos attributes(nh, pnh);

  ros::spin();

  return 0;
} // end main()