#include "ros/ros.h"
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;

struct Pose {
  double x;
  double y;
  double z;
  double theta; // rotation about world frame's y-axis
  double psi; // basically just joint 5
  double phi; // basically just joint 6
};

struct Configuration {
  double d1;
  double theta2;
  double theta3;
  double theta4;
  double theta5;
  double theta6;
};

static const double l0 = 0.4;
static const double l1 = 0.0447;
static const double l2 = 0.2798;
static const double l3 = 0.2798;
static const double l4 = 0.4447; // combined length of link 4 and swab
static const double diameter = 0.0254; // link diameter

// Admittance parameters
static const Pose m = {1, 1, 1, 100, 100, 0};
static const Pose b = {1, 1, 1, 100, 100, 0};
static const Pose k = {1, 1, 1, 100, 100, 0};

static const size_t joint_num = 6;
static const size_t steps_per_segment = 50;
static const size_t num_waypoints = 8;
static const size_t trajectory_length = steps_per_segment*num_waypoints;
static const double dt = 0.1;

// Force/torque readings in the sensor frame
// (we don't care about torque along the swab's x-axis,
// since this is likely to be negligible in practice)
static double fx = 0;
static double fy = 0;
static double fz = 0;
static double ty = 0;
static double tz = 0;
static bool first_sensor = false;
static bool first_joints = false;

static Matrix3d orientation; // current ee orientation

Matrix3d rotX(double angle) {
  double s = sin(angle);
  double c = cos(angle);

  Matrix3d rot;
  rot << 1, 0, 0,
         0, c, -s,
         0, s, c;
}

Matrix3d rotY(double angle) {
  double s = sin(angle);
  double c = cos(angle);

  Matrix3d rot;
  rot << c, 0, s,
         0, 1, 0,
         -s, 0, c;
}

Matrix3d rotZ(double angle) {
  double s = sin(angle);
  double c = cos(angle);

  Matrix3d rot;
  rot << c, -s, 0,
         s, c, 0,
         0, 0, 1;
}

// Forward kinematics for orientation of force sensor
Matrix3d fk(const Configuration &config) {
  return rotY(-config.theta2 - config.theta3 - config.theta4)*rotZ(config.theta5)*rotX(config.theta6);
}

void handle_wrench(geometry_msgs::WrenchStamped msg) {
  first_sensor = true;
  fx = max(msg.wrench.force.z - 1, 0.0);
  fy = max(msg.wrench.force.y - 1, 0.0);
  fz = max(-msg.wrench.force.x - 1, 0.0);
  ty = msg.wrench.torque.y;
  tz = -msg.wrench.torque.x;
}

void handle_joints(sensor_msgs::JointState msg) {
  first_joints = true;
  Configuration current_config;
  current_config.d1 = msg.position[0];
  current_config.theta2 = msg.position[1];
  current_config.theta3 = msg.position[2];
  current_config.theta4 = msg.position[3];
  current_config.theta5 = msg.position[4];
  current_config.theta6 = msg.position[5];

  orientation = fk(current_config);
}

void ik(Configuration &config, const Pose &pose) {
  // y position is controlled directly by prismatic joint 1
  config.d1 = pose.y - diameter - l4*sin(pose.psi);

  // Decouple effective link4
  double xp = pose.x - cos(pose.theta)*l4*cos(pose.psi) + diameter*sin(pose.theta);
  double zp = pose.z - sin(pose.theta)*l4*cos(pose.psi) - l0 - l1 - diameter*cos(pose.theta);

  // Now we're just solving RR ik
  // Always pick elbow up I guess?
  double sq_norm = xp*xp + zp*zp;
  config.theta3 = -acos((sq_norm - l2*l2 - l3*l3)/(2*l2*l3));
  config.theta2 = atan2(zp, xp) - asin(l3*sin(config.theta3)/sqrt(sq_norm));
  config.theta4 = pose.theta - config.theta2 - config.theta3;

  config.theta5 = pose.psi;

  // Rotation of swab is controlled directly by joint 6
  config.theta6 = pose.phi;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_control");
  ros::NodeHandle n("~");

  ros::Publisher cmd_pubs[joint_num];

  // Initialize command publishers
  for (size_t i = 0; i < joint_num; i++) {
    cmd_pubs[i] = n.advertise<std_msgs::Float64>("/arm/joint" + to_string(i + 1) + "_position_controller/command", 10);
  }

  // Subscribe to force sensor
  ros::Subscriber wrench_sub = n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor", 20, handle_wrench);

  // Subscribe to encoders
  ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("/arm/joint_states", 20, handle_joints);

  // Workspace waypoints
  Pose waypoints[num_waypoints + 1] = {{-diameter, 0.0254, l0 + l1 + l2 + l3 + l4, M_PI/2, 0, 0},
                                       {0.60, 0.0508, 0.75, 0, 0, 0},
                                       {0.75, 0.0508, 0.75, 0, 0, 0},
                                       {0.75, 0.0508, 0.75, 0, 0, M_PI},
                                       {0.60, 0.0508, 0.75, 0, 0, 0},
                                       {0.5, 0.0254, 0.25, -M_PI/2, 0, 0},
                                       {0.5, 0.0254, 0.1, -M_PI/2, 0, 0},
                                       {0.5, 0.0254, 0.25, -M_PI/2, 0, 0},
                                       {-diameter, 0.0254, l0 + l1 + l2 + l3 + l4, M_PI/2, 0, 0}};

  Pose trajectory[trajectory_length];
  Pose vel_trajectory[trajectory_length];
  Pose acc_trajectory[trajectory_length - 1];

  // Desired poses and velocities, post-compliance
  Pose vel_des;
  Pose pos_des;

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
      trajectory[i*steps_per_segment + step].psi = 0;
      trajectory[i*steps_per_segment + step].phi = waypoints[i].phi + step*phi_step;

      // Note that we don't comply phi. Also note that we assume zero nominal acceleration
      vel_trajectory[i*steps_per_segment + step].x = x_step/dt;
      vel_trajectory[i*steps_per_segment + step].y = y_step/dt;
      vel_trajectory[i*steps_per_segment + step].z = z_step/dt;
      vel_trajectory[i*steps_per_segment + step].theta = theta_step/dt;
      vel_trajectory[i*steps_per_segment + step].psi = 0;
    }
  }

  for (size_t i = 0; i < trajectory_length - 1; i++) {
    acc_trajectory[i].x = (vel_trajectory[i + 1].x - vel_trajectory[i].x)/dt;
    acc_trajectory[i].y = (vel_trajectory[i + 1].y - vel_trajectory[i].y)/dt;
    acc_trajectory[i].z = (vel_trajectory[i + 1].z - vel_trajectory[i].z)/dt;
    acc_trajectory[i].theta = (vel_trajectory[i + 1].theta - vel_trajectory[i].theta)/dt;
    acc_trajectory[i].psi = 0;
  }

  std_msgs::Float64 msg;
  ros::Rate r(10); 
  for (size_t i = 0; ros::ok() && i < trajectory_length; i++) {

    if (!first_sensor || !first_joints) {
      pos_des = trajectory[i];
      vel_des = vel_trajectory[i];
    } else {
      // Run compliance
      Vector3d force(fx, fy, fz);
      force = orientation*force;

      vel_des.x += dt*((force(0) - k.x*(pos_des.x - trajectory[i - 1].x) - b.x*(vel_des.x - vel_trajectory[i - 1].x))/m.x - acc_trajectory[i - 1].x); 
      pos_des.x += vel_des.x*dt;

      vel_des.y += dt*((force(1) - k.y*(pos_des.y - trajectory[i - 1].y) - b.y*(vel_des.y - vel_trajectory[i - 1].y))/m.y - acc_trajectory[i - 1].y);
      pos_des.y += vel_des.y*dt;

      vel_des.z += dt*((force(2) - k.z*(pos_des.z - trajectory[i - 1].z) - b.z*(vel_des.z - vel_trajectory[i - 1].z))/m.z - acc_trajectory[i - 1].z);
      pos_des.z += vel_des.z*dt;

      vel_des.theta += dt*((ty - k.theta*(pos_des.theta - trajectory[i - 1].theta) - b.theta*(vel_des.theta - vel_trajectory[i - 1].theta))/m.theta - acc_trajectory[i - 1].theta);
      pos_des.theta += vel_des.theta*dt;

      vel_des.psi += dt*((tz - k.psi*(pos_des.psi - trajectory[i - 1].psi) - b.psi*(vel_des.psi - vel_trajectory[i - 1].psi))/m.psi - acc_trajectory[i - 1].psi);
      pos_des.psi += vel_des.psi*dt;

      cout << vel_des.theta << endl;
    }

    Configuration config;
    ik(config, pos_des);

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

    msg.data = config.theta6;
    cmd_pubs[5].publish(msg);
    
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
