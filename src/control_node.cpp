#include "ros/ros.h"
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;

static const double l0 = 0.4;
static const double l1 = 0.0447;
static const double l2 = 0.2798;
static const double l3 = 0.2798;
static const double l4 = 0.4447; // combined length of link 4 and swab
static const double diameter = 0.0254; // link diameter

static const size_t joint_num = 6;
static const size_t steps_per_segment = 50;
static const size_t num_waypoints = 8;
static const size_t trajectory_length = steps_per_segment*num_waypoints;
static const double dt = 0.1;

struct Pose {
  double x;
  double y;
  double z;
  double theta;
  double phi;
};

struct Configuration {
  double d1;
  double theta2;
  double theta3;
  double theta4;
  double theta5;
  double phi;
};

void ik(Configuration &config, const Pose &pose) {
  // y position is controlled directly by prismatic joint 1
  config.d1 = pose.y - diameter;

  // Decouple third link
  double xp = pose.x - cos(pose.theta)*l4 + diameter*sin(pose.theta);
  double zp = pose.z - sin(pose.theta)*l4 - l0 - l1 - diameter*cos(pose.theta);

  // Now we're just solving RR ik
  // Always pick elbow up I guess?
  double sq_norm = xp*xp + zp*zp;
  config.theta3 = -acos((sq_norm - l2*l2 - l3*l3)/(2*l2*l3));
  config.theta2 = atan2(zp, xp) - asin(l3*sin(config.theta3)/sqrt(sq_norm));
  config.theta4 = pose.theta - config.theta2 - config.theta3;

  config.theta5 = 0;

  // Rotation of swab is controlled directly by joint 5 (phi)
  config.phi = pose.phi;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_control");
  ros::NodeHandle n("~");

  ros::Publisher cmd_pubs[joint_num];

  // Initialize command publishers
  for (size_t i = 0; i < joint_num; i++) {
    cmd_pubs[i] = n.advertise<std_msgs::Float64>("/arm/joint" + to_string(i + 1) + "_position_controller/command", 10);
  }

  // Workspace waypoints
  Pose waypoints[num_waypoints + 1] = {{-diameter, 0.0254, l0 + l1 + l2 + l3 + l4, M_PI/2, 0},
                                       {0.60, 0.0508, 0.75, 0, 0},
                                       {0.75, 0.0508, 0.75, 0, 0},
                                       {0.75, 0.0508, 0.75, 0, M_PI},
                                       {0.60, 0.0508, 0.75, 0, 0},
                                       {0.5, 0.0254, 0.25, -M_PI/2, 0},
                                       {0.5, 0.0254, 0.1, -M_PI/2, 0},
                                       {0.5, 0.0254, 0.25, -M_PI/2, 0},
                                       {-diameter, 0.0254, l0 + l1 + l2 + l3 + l4, M_PI/2, 0}};

  Pose trajectory[trajectory_length];

  // Generate trajectory using linear workspace interpolation between waypoints
  for (size_t i = 0; i < num_waypoints; i++) {
    double x_step = (waypoints[i + 1].x - waypoints[i].x)/((double)steps_per_segment);
    double y_step = (waypoints[i + 1].y - waypoints[i].y)/((double)steps_per_segment);
    double z_step = (waypoints[i + 1].z - waypoints[i].z)/((double)steps_per_segment);
    double theta_step = (waypoints[i + 1].theta - waypoints[i].theta)/((double)steps_per_segment);
    double phi_step = (waypoints[i + 1].phi - waypoints[i].phi)/((double)steps_per_segment);
    for (size_t step = 0; step < steps_per_segment; step++) {
      trajectory[i*steps_per_segment + step].x = waypoints[i].x + step*x_step;
      trajectory[i*steps_per_segment + step].y = waypoints[i].y + step*y_step;
      trajectory[i*steps_per_segment + step].z = waypoints[i].z + step*z_step;
      trajectory[i*steps_per_segment + step].theta = waypoints[i].theta + step*theta_step;
      trajectory[i*steps_per_segment + step].phi = waypoints[i].phi + step*phi_step;
    }
  }

  std_msgs::Float64 msg;
  ros::Rate r(10); 
  for (size_t trajectory_index = 0;
       ros::ok() && trajectory_index < trajectory_length;
       trajectory_index++) {

    Configuration config;
    ik(config, trajectory[trajectory_index]);

    msg.data = config.d1;
    cmd_pubs[0].publish(msg);

    msg.data = config.theta2 - M_PI/2;
    cmd_pubs[1].publish(msg);

    msg.data = config.theta3;
    cmd_pubs[2].publish(msg);

    msg.data = config.theta4;
    cmd_pubs[3].publish(msg);

    msg.data = config.theta5;
    cmd_pubs[4].publish(msg);

    msg.data = config.phi;
    cmd_pubs[5].publish(msg);
    
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
