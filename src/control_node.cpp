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
static const double joint6_offset = 0.0254; // link diameter

static const size_t steps_per_segment = 50;
static const size_t num_waypoints = 8;
static const size_t trajectory_length = steps_per_segment*num_waypoints;
static const size_t num_angles = 4;
static const double dt = 0.1;

void ik(array<double, num_angles> &angles, double x, double y, double theta, double phi) {
  // Decouple third link
  double xp = x - cos(theta)*l4 + joint6_offset*sin(theta);
  double yp = y - sin(theta)*l4 - l0 - l1 - joint6_offset*cos(theta);

  // Now we're just solving RR ik
  // Always pick elbow up I guess?
  double sq_norm = xp*xp + yp*yp;
  angles[1] = -acos((sq_norm - l2*l2 - l3*l3)/(2*l2*l3));
  angles[0] = atan2(yp, xp) - asin(l3*sin(angles[1])/sqrt(sq_norm));
  angles[2] = theta - angles[1] - angles[0];
  angles[3] = phi;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_control");
  ros::NodeHandle n("~");

  ros::Publisher cmd_pubs[num_angles];

  // Initialize command publishers
  for (size_t i = 0; i < num_angles; i++) {
    if (i < num_angles - 1) {
      cmd_pubs[i] = n.advertise<std_msgs::Float64>("/arm/joint" + to_string(i + 2) + "_position_controller/command", 10);
    } else {
      cmd_pubs[i] = n.advertise<std_msgs::Float64>("/arm/joint" + to_string(i + 3) + "_position_controller/command", 10);
    }
  }

  // x-y-theta-phi
  double waypoints[num_waypoints + 1][4] = {{-joint6_offset, l0 + l1 + l2 + l3 + l4, M_PI/2, 0},
                                            {0.60, 0.75, 0, 0},
                                            {0.75, 0.75, 0, 0},
                                            {0.75, 0.75, 0, M_PI},
                                            {0.60, 0.75, 0, 0},
                                            {0.5, 0.25, -M_PI/2, 0},
                                            {0.5, 0.1, -M_PI/2, 0},
                                            {0.5, 0.25, -M_PI/2, 0},
                                            {-joint6_offset, l0 + l1 + l2 + l3 + l4, M_PI/2, 0}};

  array<array<double, num_angles>, trajectory_length> trajectory;

  // Generate trajectory using linear workspace interpolation between waypoints
  for (size_t i = 0; i < num_waypoints; i++) {
    double x_step = (waypoints[i + 1][0] - waypoints[i][0])/((double)steps_per_segment);
    double y_step = (waypoints[i + 1][1] - waypoints[i][1])/((double)steps_per_segment);
    double theta_step = (waypoints[i + 1][2] - waypoints[i][2])/((double)steps_per_segment);
    double phi_step = (waypoints[i + 1][3] - waypoints[i][3])/((double)steps_per_segment);
    for (size_t angle = 0; angle < steps_per_segment; angle++) {
      array<double, num_angles> angles;
      ik(angles,
         waypoints[i][0] + ((double)angle)*x_step,
         waypoints[i][1] + ((double)angle)*y_step,
         waypoints[i][2] + ((double)angle)*theta_step,
         waypoints[i][3] + ((double)angle)*phi_step);
      trajectory[i*steps_per_segment + angle][0] = angles[0] - M_PI/2;
      trajectory[i*steps_per_segment + angle][1] = angles[1];
      trajectory[i*steps_per_segment + angle][2] = angles[2];
      trajectory[i*steps_per_segment + angle][3] = angles[3];
    }
  }

  std_msgs::Float64 msg;
  ros::Rate r(10); 
  for (size_t trajectory_index = 0;
       ros::ok() && trajectory_index < trajectory_length;
       trajectory_index++) {
    for (size_t i = 0; i < num_angles; i++) {
      msg.data = trajectory[trajectory_index][i];
      cmd_pubs[i].publish(msg);
    }
    
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
