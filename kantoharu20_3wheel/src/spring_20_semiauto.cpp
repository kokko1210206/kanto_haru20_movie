#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <math.h>
float x = 0.388; //車軸中心位置[m] 
float y = 3.112; //車軸中心位置[m
float vd = 0.15; //並進速度目標値[m/s] 
float xd = 15.6; //x軸方向目標位置[m] 
float v1 = 0.0; //並進速度[m/s] 
float v2 = 0.0; //旋回角速度[rad/s] 
float v1i = 0.0; //v1の偏差の積算値
double yr;
float e; //偏差
float e0 = yr; //１時刻前の偏差
float ei = 0.0; //偏差の積算値
float ed = 0.0; //偏差の微分値
float u1, u2; //並進，旋回角速度入力[m/s] 
long sec;
long nsec;
long time_new; //時刻[ms] 
long time_old = 0; //一回前の時刻[ms] 
long dt = 0; //サンプリング時間[ms]
static float kp = 5.5;//比例ゲイン
static float ki = 0;//積分ゲイン
static float kd = 3;//微分ゲイン
static float xkp = 8;//並進方向の比例ゲイン
static float xki = 2;//並進方向の積分ゲイン

class Sp_auto_node
{
public:
    Sp_auto_node();

private:
    void pidcmdcallback(const gazebo_msgs::ModelStates::ConstPtr& pose_msg);
    ros::NodeHandle nh_;

    ros::Publisher _vel_pub;
    ros::Subscriber  _odom_sub;
};



Sp_auto_node::Sp_auto_node()
{
    _vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    _odom_sub = nh_.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 50, &Sp_auto_node::pidcmdcallback,this);
    

}

void Sp_auto_node::pidcmdcallback(const gazebo_msgs::ModelStates::ConstPtr& pose_msg)
{
    geometry_msgs::Twist twist;
    x = pose_msg->pose[2].position.x;
    y = pose_msg->pose[2].position.y;
    yr = (-1.0/10.5)*x*x + 3.03;
    v1 = pose_msg->twist[2].linear.x;
    v2 = pose_msg->twist[2].angular.z;
    time_new = ros::Time::now().sec*1000 + ros::Time::now().nsec/1000000;
    dt = time_new - time_old;
    time_old = time_new;
    v1i = v1i + (vd - v1)*float(dt)/1000.0;  
    u1 =xkp*(vd - v1)+ xki*v1i;    //並進方向速度入力(PI control)   
    e = (yr - y);   
    ei = ei + e*float(dt)/1000.0;   
    ed = (e - e0)/float(dt)*1000.0;   
    u2 = kp*e + ki*ei + kd*ed;     //旋回方向速度入力(PID control)   
    e0 = e;      
    if(x > xd){
        u1 = 0;
        u2 = 0;   
    }
    twist.linear.x = u1;
    twist.linear.y = 0;
    twist.angular.z = u2;
    ROS_INFO("x,y,yr,u1,u2,%f,%f, %lf,%f,%f",x,y,yr,u1,u2);
    _vel_pub.publish(twist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spring_20_semiauto");

    Sp_auto_node sp_auto_node;

    ros::spin();
}
