//
// Created by qiayuan on 2/6/21.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

    HeroChassisController::~HeroChassisController() {
        sub_command_.shutdown();
        odom_pub.shutdown();
    }

    bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                     ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        pid_controller1_.init(ros::NodeHandle(controller_nh, "pid1"));
        pid_controller2_.init(ros::NodeHandle(controller_nh, "pid2"));
        pid_controller3_.init(ros::NodeHandle(controller_nh, "pid3"));
        pid_controller4_.init(ros::NodeHandle(controller_nh, "pid4"));

        controller_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<control_msgs::JointControllerState >>(
                controller_nh, "state", 1);

        sub_command_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &HeroChassisController::get_vel_cmd,
                                                               this);;

        odom_pub = root_nh.advertise<nav_msgs::Odometry>("/odom", 50);

        return true;
    }

    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {

        now = time;

        vel_cur[1] = front_right_joint_.getVelocity();
        vel_cur[2] = front_left_joint_.getVelocity();
        vel_cur[3] = back_left_joint_.getVelocity();
        vel_cur[4] = back_right_joint_.getVelocity();




        transform_then_pub();
        //compute the command speed of wheel
        compute_the_cmd();

        for (int i = 1; i < 5; i++) {
            error[i] = vel_cmd[i] - vel_cur[i];
        }


        front_right_joint_.setCommand(pid_controller1_.computeCommand(error[1], period));
        front_left_joint_.setCommand(pid_controller2_.computeCommand(error[2], period));
        back_left_joint_.setCommand(pid_controller3_.computeCommand(error[3], period));
        back_right_joint_.setCommand(pid_controller4_.computeCommand(error[4], period));

        if (loop_count_ % 10 == 0) {
            if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
                controller_state_publisher_->msg_.header.stamp = now;
                controller_state_publisher_->msg_.set_point = vel_cmd[1];
                controller_state_publisher_->msg_.process_value = vel_cur[1];
                controller_state_publisher_->msg_.error = error[1];
                controller_state_publisher_->msg_.time_step = period.toSec();
                controller_state_publisher_->msg_.command = pid_controller1_.computeCommand(error[1], period);

                double dummy;
                bool antiwindup;
                pid_controller1_.getGains(controller_state_publisher_->msg_.p,
                                          controller_state_publisher_->msg_.i,
                                          controller_state_publisher_->msg_.d,
                                          controller_state_publisher_->msg_.i_clamp,
                                          dummy,
                                          antiwindup);
                controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
                controller_state_publisher_->unlockAndPublish();
            }
        }
        loop_count_++;
        last_time = now;
    }

    void HeroChassisController::get_vel_cmd(const geometry_msgs::TwistConstPtr &msg) {
        Vxe = msg->linear.x;
        Vye = msg->linear.y;
        yawe = msg->angular.z;
    }

    void HeroChassisController::compute_the_cmd() {
        vel_cmd[1] = (Vxe + Vye + yawe * (a + b) / 2) / RADIUS;
        vel_cmd[2] = (Vxe - Vye - yawe * (a + b) / 2) / RADIUS;
        vel_cmd[3] = (Vxe + Vye - yawe * (a + b) / 2) / RADIUS;
        vel_cmd[4] = (Vxe - Vye + yawe * (a + b) / 2) / RADIUS;
    };

    void HeroChassisController::compute_the_cmd_rot() {
        compute_the_cmd();

        for (int i = 1; i < 5; i++) {
            cmd_rot[i] = vel_cmd[i] / 2 / (2 * asin(1));
            ROS_INFO("cmo rot is:%lf", cmd_rot[i]);
        }

    }

    void HeroChassisController::compute_the_cur_vel() {
        Vx_cur = (vel_cur[1] + vel_cur[2] + vel_cur[3] + vel_cur[4]) * RADIUS / 4;
        Vy_cur = (vel_cur[1] - vel_cur[2] + vel_cur[3] - vel_cur[4]) * RADIUS / 4;
        yaw_cur = (vel_cur[1] - vel_cur[2] - vel_cur[3] + vel_cur[4]) * RADIUS / 2 / (a + b);

    }

    void HeroChassisController::transform_then_pub() {

        now = ros::Time::now();
        last_time = ros::Time::now();

        compute_the_cur_vel();

        now = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        dt = (now - last_time).toSec();
        double delta_x = (Vx_cur * cos(th) - Vy_cur * sin(th)) * dt;
        double delta_y = (Vx_cur * sin(th) + Vy_cur * cos(th)) * dt;
        double delta_th = yaw_cur * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = now;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = Vx_cur;
        odom.twist.twist.linear.y = Vy_cur;
        odom.twist.twist.angular.z = yaw_cur;

        //publish the message
        odom_pub.publish(odom);

        last_time = now;
    }

    void HeroChassisController::transform_the_frame() {


    }
}
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)

