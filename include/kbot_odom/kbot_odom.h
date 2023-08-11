#ifndef KBOT_ODOM_H__
#define KBOT_ODOM_H__

#include <math.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <kbot_msgs/lowlevel.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define WHEEL_RADIUS    0.034
#define LEFT            0
#define RIGHT           1
#define DEG2RAD(x)      (x * 0.01745329252) // PI/180
#define RPM2MPS(x)      (x * 0.003560471674) // V = (2*PI*r)/60 * N

class KBOTODOM{
    public:
        KBOTODOM();
        ~KBOTODOM();
        bool init();
        bool update();
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;

        ros::Time last_time_lw_;
        ros::Time prev_update_time_;

        ros::Publisher joint_states_pub_;
        ros::Publisher odom_pub_;

        ros::Subscriber kbot_lw_data_sub_;
        ros::Subscriber reset_odom_;

        sensor_msgs::JointState joint_states_;
        nav_msgs::Odometry odom_;
        tf::TransformBroadcaster tf_broadcaster_;

        double wheel_speed_cmd_[2];
        double yaw_data_;
        double lw_data_timeout_;

        float odom_pose_[3];
        float odom_vel_[3];
        double pose_cov_[36];

        std::string joint_states_name_[2];

        double last_position_[2];
        double last_velocity_[2];

        void commandDataCallback(const kbot_msgs::lowlevelConstPtr lw_msg);
        void commandResetCallback(const std_msgs::EmptyConstPtr reset_odom);
        bool updateOdometry(ros::Duration diff_time);
        void updateJoint(void);
        void updateTF(geometry_msgs::TransformStamped& odom_tf);
};

#endif