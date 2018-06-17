#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <vector>

#include <ros/console.h>
#include <ros/master.h>
#include <ros/ros.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

const char NODE_SIMULATOR[] = "/simulator";
const char NODE_STAGE[] = "/stage";

double sensor_range_;
double sensor_range2_;
double sensor_fov_;
double sensor_fov_half_;
double tag_visibility_fov_;
double tag_visibility_fov_half_;

ros::Publisher pub_tag_detections_;

std::map<int, std::shared_ptr<tf2::Transform>> tag_positions_;
/* sensor_msgs::LaserScanConstPtr last_scan_; */

void cbNewPose(const nav_msgs::Odometry& msg) {
  // Gather some useful information from the robot's pose
  tf2::Transform pose_robot, pose_to_tag;
  tf2::Vector3 diff_dist, yaw_robot, yaw_tag, zero_vector(1, 0, 0);
  tf2::fromMsg(msg.pose.pose, pose_robot);
  yaw_robot = tf2::quatRotate(pose_robot.getRotation(), zero_vector);

  // Look through all known tag_positions for a valid observation
  apriltags_ros::AprilTagDetectionArray atda;
  for (const std::pair<int, std::shared_ptr<tf2::Transform>>& tp :
       tag_positions_) {
    // Apply the distance criteria first
    diff_dist = (tp.second)->getOrigin() - pose_robot.getOrigin();
    if (diff_dist.length2() > sensor_range2_) continue;
    /* ROS_WARN("Tag %d is within %fm of robot", tp.first, sensor_range_); */

    // Then within robot's field of view
    if (diff_dist.angle(yaw_robot) > sensor_fov_half_) continue;
    /* ROS_WARN("Tag %d is within the %fdeg robot FOV", tp.first, */
    /*          sensor_fov_ * 180.0 / M_PI); */

    // Finally, within tag's "field of view"
    yaw_tag = tf2::quatRotate((tp.second)->getRotation(), zero_vector);
    if ((-1 * diff_dist).angle(yaw_tag) > tag_visibility_fov_half_) continue;
    /* ROS_WARN("Tag %d is within the %fdeg tag FOV", tp.first, */
    /*          tag_visibility_fov_ * 180.0 / M_PI); */

    // TODO handle if behind an obstacle

    // We have a valid observation, append to the message
    apriltags_ros::AprilTagDetection atd;
    atd.id = tp.first;
    atd.size = 0.163513;  // Copied from real node
    tf2::toMsg(
        tf2::Stamped<tf2::Transform>(pose_robot.inverseTimes(*(tp.second)),
                                     ros::Time::now(), "/laser"),
        atd.pose);
    atda.detections.push_back(atd);
  }
  pub_tag_detections_.publish(atda);
}

/* void cbNewScan(const sensor_msgs::LaserScanConstPtr msgPtr) { */
/*   last_scan_ = msgPtr; */
/* } */

void refreshTagPoses() {
  // Get a list of all topics
  ros::master::V_TopicInfo tis;
  ros::master::getTopics(tis);

  // Go through each topic, pulling out odometry if it is a tag pose
  std::regex topic_regex_("/tag_(\\d+)/base_pose_ground_truth");
  std::smatch matches;
  ROS_INFO("Looking for tags in published topics...");
  for (const ros::master::TopicInfo& ti : tis) {
    if (std::regex_search(ti.name, matches, topic_regex_)) {
      int tag_id = std::stoi(matches[1].str());
      tag_positions_[tag_id] = std::make_shared<tf2::Transform>();
      nav_msgs::OdometryConstPtr temp =
          ros::topic::waitForMessage<nav_msgs::Odometry>(ti.name);
      tf2::fromMsg(temp->pose.pose, *(tag_positions_[tag_id]));
    }
  }
  ROS_INFO("Found and saved the position of %d tags",
           (int)tag_positions_.size());
}

int main(int argc, char** argv) {
  // Initialise the node
  ros::init(argc, argv, "find_tags_simulated");
  ros::NodeHandle nh, nh_private("~");
  nh_private.param("sensor_range", sensor_range_, 3.0);
  nh_private.param("sensor_fov_deg", sensor_fov_, 350.0);
  nh_private.param("tag_visibility_fov", tag_visibility_fov_, 90.0);
  sensor_range2_ = sensor_range_ * sensor_range_;
  sensor_fov_ *= M_PI / 180.0;
  sensor_fov_half_ = 0.5 * sensor_fov_;
  tag_visibility_fov_ *= M_PI / 180.0;
  tag_visibility_fov_half_ = 0.5 * tag_visibility_fov_;

  // Wait until the required simulator nodes are present
  std::vector<std::string> nodes;
  bool nodes_found = false;
  while (ros::ok() && !nodes_found) {
    ros::master::getNodes(nodes);
    nodes_found =
        std::any_of(nodes.begin(), nodes.end(),
                    [](std::string s) { return s == NODE_SIMULATOR; }) &&
        std::any_of(nodes.begin(), nodes.end(),
                    [](std::string s) { return s == NODE_STAGE; });
    ros::Duration(2).sleep();
    if (!nodes_found) {
      ROS_WARN("Waiting for nodes %s and %s to become available...",
               NODE_SIMULATOR, NODE_STAGE);
    }
  }

  // Get the tag ground truth poses once before beginning (we make the safe
  // assumption that they are static)
  refreshTagPoses();

  // Configure all publishers and subscribers, letting callbacks control the
  // node from now on
  pub_tag_detections_ = nh.advertise<apriltags_ros::AprilTagDetectionArray>(
      "/tag_detections", 10);
  ros::Subscriber sub_robot_pose =
      nh.subscribe("/guiabot/base_pose_ground_truth", 100, cbNewPose);
  /* ros::Subscriber sub_laser = nh.subscribe("/scan_laser_fixed", 100,
   * cbNewScan); */

  ros::spin();
  return 0;
}
