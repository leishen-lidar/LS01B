/*******************************************************
@company: Copyright (C) 2018, Leishen Intelligent System
@product: LS01B
@filename: ls01b_node.h
@brief:
@version:       date:       author:     comments:
@v1.0           18-8-28     fu          new
*******************************************************/
#ifndef LS01B_NODE_H
#define LS01B_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "ls01b_v2/ls01b.h"

namespace ls {
typedef struct{
  double start;
  double end;
}DisableAngle;

class LS01B_Node{
public:
  LS01B_Node();
  ~LS01B_Node();
  void run();

private:
  void initParam();

  void publishScan(const ros::TimerEvent&);

  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Timer timer_;

  LS01B *ls01b_;
  std::string serial_port_;
  int baud_rate_;
  double angle_resolution_;
  std::string scan_topic_;
  std::string frame_id_;

  DisableAngle dis_angle_[5];
  double robot_radius_;
  double center_x_;
  double center_y_;
  double angle_disable_min_0;
  double angle_disable_max_0;
  double angle_disable_min_1;
  double angle_disable_max_1;
  double angle_disable_min_2;
  double angle_disable_max_2;
  double angle_disable_min_3;
  double angle_disable_max_3;
  double angle_disable_min_4;
  double angle_disable_max_4;

  bool is_shuttdown_;
};
}

#endif // LS01B_NODE_H
