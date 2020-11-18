#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class odom_trans_node
{
public:
    odom_trans_node(void);

private:
    void odom_trans_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    ros::NodeHandle nh_;

    tf::TransformBroadcaster odom_broad,odom_broada;
    ros::Subscriber odom_sub;
    geometry_msgs::TransformStamped map_to_odom , odom_to_robot;
};

odom_trans_node::odom_trans_node(void)
{
    odom_sub = nh_.subscribe<geometry_msgs::PoseStamped>("odom_pose",10, &odom_trans_node::odom_trans_callback, this);
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    odom_to_robot.header.frame_id = "odom";
    odom_to_robot.child_frame_id = "base_link";
}

void odom_trans_node::odom_trans_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    this->map_to_odom.transform.translation.x = pose_msg->pose.position.x + 0.172085*cos(M_PI*180.0/79.68);
    this->map_to_odom.transform.translation.y = pose_msg->pose.position.y + 0.172085*sin(M_PI*180.0/79.68);
    this->map_to_odom.transform.translation.z = 0.0;
    this->map_to_odom.header.stamp = pose_msg->header.stamp;
    geometry_msgs::Quaternion odom_quat_map = tf::createQuaternionMsgFromYaw(pose_msg->pose.orientation.z);
    this->map_to_odom.transform.rotation = odom_quat_map;

    this->odom_to_robot.transform.translation.x = -0.172085*cos(M_PI*180.0/79.68);
    this->odom_to_robot.transform.translation.y = -0.172085*sin(M_PI*180.0/79.68);
    this->odom_to_robot.transform.translation.z = 0.0;
    this->odom_to_robot.header.stamp = pose_msg->header.stamp;
    geometry_msgs::Quaternion odom_quat_odom = tf::createQuaternionMsgFromYaw(0.0);
    this->odom_to_robot.transform.rotation = odom_quat_odom;

    odom_broada.sendTransform(map_to_odom);
    odom_broad.sendTransform(odom_to_robot);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_transform");

    odom_trans_node *Odom_trans_node = new odom_trans_node();

    ros::spin();
   
}



    
