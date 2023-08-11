#include <kbot_odom/kbot_odom.h>

KBOTODOM::KBOTODOM()
: nh_priv_("~")
{
    // Init kbot odom node
    bool init_result = init();
    ROS_ASSERT(init_result);
}

KBOTODOM::~KBOTODOM()
{

}

bool KBOTODOM::init()
{
    nh_.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("wheel_left_joint"));
    nh_.param("wheel_right_joint_name", joint_states_name_[RIGHT],  std::string("wheel_right_joint"));
    nh_.param("joint_states_frame", joint_states_.header.frame_id, std::string("base_footprint"));
    nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
    nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));
    
    wheel_speed_cmd_[LEFT]  = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
    yaw_data_               = 0.0;
    lw_data_timeout_        = 1.0;
    last_position_[LEFT]    = 0.0;
    last_position_[RIGHT]   = 0.0;

    double pcov[36] = { 0.1, 0,   0,   0,   0, 0,
                        0, 0.1,   0,   0,   0, 0,
                        0,   0, 1e6,   0,   0, 0,
                        0,   0,   0, 1e6,   0, 0,
                        0,   0,   0,   0, 1e6, 0,
                        0,   0,   0,   0,   0, 0.2};
    memcpy(&(odom_.pose.covariance),pcov,sizeof(double)*36);
    memcpy(&(odom_.twist.covariance),pcov,sizeof(double)*36);

    odom_pose_[0] = 0.0;
    odom_pose_[1] = 0.0;
    odom_pose_[2] = 0.0;

    odom_vel_[0] = 0.0;
    odom_vel_[1] = 0.0;
    odom_vel_[2] = 0.0;

    joint_states_.name.push_back(joint_states_name_[LEFT]);
    joint_states_.name.push_back(joint_states_name_[RIGHT]);
    joint_states_.position.resize(2,0.0);
    joint_states_.velocity.resize(2,0.0);
    joint_states_.effort.resize(2,0.0);

    // initialize publishers
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 100);
    odom_pub_         = nh_.advertise<nav_msgs::Odometry>("/odom", 100);

    // initialize subscribers
    kbot_lw_data_sub_   = nh_.subscribe("/kbot/micro/data", 100,  &KBOTODOM::commandDataCallback, this);
    reset_odom_         = nh_.subscribe("/kbot/reset", 100, &KBOTODOM::commandResetCallback, this);

    prev_update_time_ = ros::Time::now();

    return true;
}

void KBOTODOM::commandDataCallback(const kbot_msgs::lowlevelConstPtr lw_msg)
{
    last_time_lw_ = ros::Time::now();

    wheel_speed_cmd_[LEFT]  = RPM2MPS((double)lw_msg->speed_fb[LEFT]);
    wheel_speed_cmd_[RIGHT] = RPM2MPS((double)lw_msg->speed_fb[RIGHT]);
    yaw_data_               = (double)lw_msg->theta;
}

void KBOTODOM::commandResetCallback(const std_msgs::EmptyConstPtr reset_odom)
{
    wheel_speed_cmd_[LEFT]  = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
    yaw_data_               = 0.0;
    lw_data_timeout_        = 1.0;
    last_position_[LEFT]    = 0.0;
    last_position_[RIGHT]   = 0.0;

    double pcov[36] = { 0.1, 0,   0,   0,   0, 0,
                        0, 0.1,   0,   0,   0, 0,
                        0,   0, 1e6,   0,   0, 0,
                        0,   0,   0, 1e6,   0, 0,
                        0,   0,   0,   0, 1e6, 0,
                        0,   0,   0,   0,   0, 0.2};
    memcpy(&(odom_.pose.covariance),pcov,sizeof(double)*36);
    memcpy(&(odom_.twist.covariance),pcov,sizeof(double)*36);

    odom_pose_[0] = 0.0;
    odom_pose_[1] = 0.0;
    odom_pose_[2] = 0.0;

    odom_vel_[0] = 0.0;
    odom_vel_[1] = 0.0;
    odom_vel_[2] = 0.0;

    ROS_INFO("Reset Odometry.");
}

bool KBOTODOM::updateOdometry(ros::Duration diff_time)
{
    double wheel_l, wheel_r;
    double delta_s, theta, delta_theta;
    double v[2], w[2];
    static double last_theta = 0.0;

    wheel_l = wheel_r       = 0.0;
    delta_s = theta = delta_theta  = 0.0;

    v[LEFT]  = wheel_speed_cmd_[LEFT];
    w[LEFT]  = v[LEFT] / WHEEL_RADIUS;
    v[RIGHT] = wheel_speed_cmd_[RIGHT];
    w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

    wheel_l = w[LEFT]   * diff_time.toSec();
    wheel_r = w[RIGHT]  * diff_time.toSec();

    if(isnan(wheel_l))  wheel_l = 0.0;
    if(isnan(wheel_r))  wheel_r = 0.0;

    delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
    theta       = DEG2RAD(yaw_data_);
    delta_theta = theta - last_theta;
    
    // compute odometric pose
    odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
    odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
    odom_pose_[2] += delta_theta;

    // compute odometric instantaneouse velocity
    odom_vel_[0] = delta_s / diff_time.toSec();     // v
    odom_vel_[1] = 0.0;
    odom_vel_[2] = delta_theta / diff_time.toSec(); // w

    odom_.pose.pose.position.x = odom_pose_[0];
    odom_.pose.pose.position.y = odom_pose_[1];
    odom_.pose.pose.position.z = 0;
    odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose_[2]);

    // We should update the twist of the odometry
    odom_.twist.twist.linear.x  = odom_vel_[0];
    odom_.twist.twist.angular.z = odom_vel_[2];

    // Update Data
    last_velocity_[LEFT]  = w[LEFT];
    last_velocity_[RIGHT] = w[RIGHT];
    last_position_[LEFT]  += wheel_l;
    last_position_[RIGHT] += wheel_r;
    last_theta = theta;

    return true;
}

void KBOTODOM::updateJoint(void)
{
    joint_states_.position[LEFT]  = last_position_[LEFT];
    joint_states_.position[RIGHT] = last_position_[RIGHT];
    joint_states_.velocity[LEFT]  = last_velocity_[LEFT];
    joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];
}

void KBOTODOM::updateTF(geometry_msgs::TransformStamped& odom_tf)
{
    odom_tf.header = odom_.header;
    odom_tf.child_frame_id = odom_.child_frame_id;
    odom_tf.transform.translation.x = odom_.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_.pose.pose.position.z;
    odom_tf.transform.rotation = odom_.pose.pose.orientation;
}

bool KBOTODOM::update()
{
    ros::Time time_now = ros::Time::now();
    ros::Duration step_time = time_now - prev_update_time_;
    prev_update_time_ = time_now;

    // zero-ing after timeout
    if((time_now - last_time_lw_).toSec() > lw_data_timeout_)
    {
        wheel_speed_cmd_[LEFT]  = 0.0;
        wheel_speed_cmd_[RIGHT] = 0.0;
    }

    // odom
    updateOdometry(step_time);
    odom_.header.stamp = time_now;
    odom_pub_.publish(odom_);

    // joint_states
    updateJoint();
    joint_states_.header.stamp = time_now;
    joint_states_pub_.publish(joint_states_);

    // tf
    geometry_msgs::TransformStamped odom_tf;
    updateTF(odom_tf);
    tf_broadcaster_.sendTransform(odom_tf);

    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "kbot_odom_node");
    KBOTODOM kbot_odom_;

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        kbot_odom_.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}