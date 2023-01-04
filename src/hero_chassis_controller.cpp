//
// Created by qiayuan on 2/6/21.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {
bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                   ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  front_left_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");

  pid_controller1_.init(ros::NodeHandle(controller_nh,"pid1"));
    pid_controller2_.init(ros::NodeHandle(controller_nh,"pid2"));
    pid_controller3_.init(ros::NodeHandle(controller_nh,"pid3"));
    pid_controller4_.init(ros::NodeHandle(controller_nh,"pid4"));

    sub_command_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &HeroChassisController::get_vel_cmd,this);

  return true;
}

void HeroChassisController::get_vel_cmd(const geometry_msgs::TwistConstPtr &msg) {
    Vxe=msg->linear.x;
    Vye=msg->linear.y;
    yawe=msg->angular.z;
}

    void HeroChassisController::compute_the_cmd(ros::NodeHandle &controller_nh) {
        vel_cmd[1] = (Vxe + Vye + yawe * (a + b) / 2) / r;
        vel_cmd[2] = (Vxe - Vye - yawe * (a + b) / 2) / r;
        vel_cmd[3] = (Vxe + Vye - yawe * (a + b) / 2) / r;
        vel_cmd[4] = (Vxe - Vye + yawe * (a + b) / 2) / r;
};



    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {

    vel_cur[1]=front_right_joint_.getVelocity();
    vel_cur[2]=front_left_joint_.getVelocity();
    vel_cur[3]=back_left_joint_.getVelocity();
    vel_cur[4]=back_right_joint_.getVelocity();

    error[1]=vel_cmd[1]-vel_cur[1];
    error[2]=vel_cmd[2]-vel_cur[2];
    error[3]=vel_cmd[3]-vel_cur[3];
    error[4]=vel_cmd[4]-vel_cur[4];


    front_right_joint_.setCommand(pid_controller1_.computeCommand(error[1], period));
  front_left_joint_.setCommand(pid_controller2_.computeCommand(error[2], period));
  back_left_joint_.setCommand(pid_controller3_.computeCommand(error[3], period));
  back_right_joint_.setCommand(pid_controller4_.computeCommand(error[4], period));
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}
