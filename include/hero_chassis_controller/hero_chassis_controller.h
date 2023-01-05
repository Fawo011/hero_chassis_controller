//
// Created by qiayuan on 2/6/21.
//

#ifndef SIMPLE_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define SIMPLE_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <math.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
 private:

    std::unique_ptr<realtime_tools::RealtimePublisher<
            control_msgs::JointControllerState> > controller_state_publisher_ ;

 control_toolbox::Pid pid_controller1_;   //left_front
 control_toolbox::Pid pid_controller2_;   //right_front
 control_toolbox::Pid pid_controller3_;   //left_back
 control_toolbox::Pid pid_controller4_;   //right_back

    tf::TransformBroadcaster odom_broadcaster;


 void get_vel_cmd(const geometry_msgs::TwistConstPtr &msg);

 void compute_the_cmd();
    void compute_the_cmd_rot();
    void compute_the_cur_vel();
    void transform_then_pub();

    //command vel
    double vel_cmd[5]{0.0,0.0,0.0,0.0,0.0};
    //vel after giving PID
     double vel_cur[5]{};
     //e of PID
     double error[5]{0.0,0.0,0.0,0.0};

    double Vxe{0.0}, Vye{0.0}, yawe{0.0};
    double Vxc{},Vyc{},yawc{};
    double a{0.320},b{0.410},r{0.02};
    double cmd_rot[5]{};
    double dt{};
    double x{0.0}, y{0.0}, th{0.0};
    double Vx_cur{},Vy_cur{},yaw_cur{};

    ros::Subscriber sub_command_;
    ros::Publisher odom_pub;
    int loop_count_{};

    ros::Time last_time;
    ros::Time now;



};
}// namespace simple_chassis_controller

#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
