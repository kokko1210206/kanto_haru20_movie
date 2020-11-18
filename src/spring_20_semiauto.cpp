#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/Joy.h>
#include <vector>
#include <tf/transform_listener.h>
#include <string>

enum class go_status
	:int
	{
		disable, x_axis_go, y_axis_go
	};

class Sp_auto_node
{
public:
    Sp_auto_node();
private:
    void pid_start(const ros::TimerEvent &event);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    tf::TransformListener ln;
    geometry_msgs::PoseStamped source_pose;
    geometry_msgs::PoseStamped target_pose;
    void go_x_set(float xd,float vd,float yr,float kp,float ki,float kd,float go_kp,float go_ki);
    void go_y_set(float yd,float vd, float xr,float kp,float ki,float kd,float go_kp,float go_ki);
    void reset(void);
    ros::NodeHandle nh_;
    double time_old;
    ros::Publisher _vel_pub;
    ros::Subscriber  _odom_sub;
    ros::Subscriber _joy_sub;
    static int ButtonA;
    static int ButtonLB;
    static int ButtonRB;
    float x; 
    float y;
    float x_old;
    float y_old;
    float x_begin;
    float y_begin;
    std::vector<float> vd; //target velocity [m/s] 
    std::vector<double> xd; //target x coodinate [m]
    std::vector<double> yd; //target y coodinate [m]
    std::vector<int> x_or_y; //go straight x or y axis
    std::vector<double> yr;
    std::vector<double> xr;
    float v1; 
    float v2; 
    float v1i;
    
    float e;
    std::vector<float> e0;
    float ei;
    float ed;
    float u1, u2; 
    long sec;
    long nsec;
    double time_new; //時刻[ms]  
    double dt;
    int goal_flag;
    int time_count;
    std::vector<float> kp;
    std::vector<float> ki;
    std::vector<float> kd;
    std::vector<float> go_kp;
    std::vector<float> go_ki;
    bool _ButtonLB_b_last;
    bool _ButtonRB_b_last;
    ros::Timer timer;
};

int Sp_auto_node::ButtonA = 1;
int Sp_auto_node::ButtonLB = 4;
int Sp_auto_node::ButtonRB = 5;


Sp_auto_node::Sp_auto_node()
{
    _vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    _joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Sp_auto_node::joyCallback, this);
    nh_.getParam("ButtonA", ButtonA);
    nh_.getParam("ButtonLB", ButtonLB);
    nh_.getParam("ButtonRB", ButtonRB);
    goal_flag = 0;
    time_count= 0;
    x_or_y.push_back((int)go_status::disable);
    this->timer = nh_.createTimer(ros::Duration(0.01),&Sp_auto_node::pid_start,this);
    x_old = 0.0;
    y_old = 0.0;
}

void Sp_auto_node::reset(void)
{
    xd.clear();
    yd.clear();
    x_or_y.clear();
    x_or_y.push_back((int)go_status::disable);
    vd.clear();
    v1i = 0.0;
    xr.clear();
    yr.clear();
    e0.clear();
    ei = 0;
    ed = 0;
    kp.clear();//比例ゲイン
    ki.clear();//積分ゲイン
    kd.clear();//微分ゲイン
    go_kp.clear();//並進方向の比例ゲイン
    go_ki.clear();//並進方向の積分ゲイン
    goal_flag = 0;
    time_count = 0;
    u1 = 0;
    u2 = 0;
    v1 = 0;
    ROS_INFO("reset");
}    


void Sp_auto_node::go_x_set(float xd,float vd,float yr,float kp,float ki,float kd,float go_kp,float go_ki){
    this->xd.push_back(xd);
    x_or_y.push_back((int)go_status::x_axis_go); 
    this->vd.push_back(vd);
    v1i = 0.0;
    this->yr.push_back(yr);
    e0.push_back(yr);
    ei = 0;
    ed = 0;
    this->kp.push_back(kp);//比例ゲイン
    this->ki.push_back(ki);//積分ゲイン
    this->kd.push_back(kd);//微分ゲイン
    this->go_kp.push_back(go_kp);//並進方向の比例ゲイン
    this->go_ki.push_back(go_ki);//並進方向の積分ゲイン
 }

void Sp_auto_node::go_y_set(float yd,float vd, float xr,float kp,float ki,float kd,float go_kp,float go_ki){
    this->xd.push_back(yd);
    x_or_y.push_back((int)go_status::y_axis_go);
    this->vd.push_back(vd);
    v1i = 0.0;
    this->xr.push_back(xr);
    e0.push_back(xr);
    ei = 0;
    ed = 0;
    this->kp.push_back(kp);//比例ゲイン
    this->ki.push_back(ki);//積分ゲイン
    this->kd.push_back(kd);//微分ゲイン
    this->go_kp.push_back(go_kp);//並進方向の比例ゲイン
    this->go_ki.push_back(go_ki);//並進方向の積分ゲイン
}
 
void Sp_auto_node::pid_start(const ros::TimerEvent&)
{
    if(this->x_or_y.at(goal_flag) == (int)go_status::disable){
    	ROS_INFO("%ld",x_or_y.size());
	}
    else{
        ROS_INFO("count start");
	source_pose.header.frame_id="map";
	source_pose.header.stamp=ros::Time(0);
	source_pose.pose.orientation.w=1.0;
	std::string target_frame="base_link";

    if(time_count == 0){
    	ROS_INFO("enable");
    	time_count++;
    	try{
    		ln.waitForTransform(source_pose.header.frame_id, target_frame, ros::Time::now(), ros::Duration(0.1));
		ln.transformPose(target_frame, source_pose, target_pose);

      		ROS_INFO("x:%+5.2f, y:%+5.2f,z:%+5.2f,%d",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z,target_pose.header.stamp.nsec);
    	   }
    	catch(...){
        	ROS_INFO("tf error");
    	          }
    	x_begin = target_pose.pose.position.x;
    	y_begin = target_pose.pose.position.y;
    	x_old = x_begin;
    	y_old = y_begin;
    	time_old = target_pose.header.stamp.sec*1000.0 + target_pose.header.stamp.nsec/1000000.0;}
    	else if (time_count > 0){
    		ROS_INFO("OK");
    		try{
      			ln.waitForTransform(source_pose.header.frame_id, target_frame, ros::Time::now(), ros::Duration(0.1));
      			ln.transformPose(target_frame, source_pose, target_pose);

      			ROS_INFO("x:%+5.2f, y:%+5.2f,z:%+5.2f,%d",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z,target_pose.header.stamp.nsec);
    		   }
    		catch(...){
       			ROS_INFO("tf error");
    		           }
    		ROS_INFO("yes");
    		geometry_msgs::Twist twist;
    		x = target_pose.pose.position.x;
    		y = target_pose.pose.position.y;
    		ROS_INFO("position OK");
    		time_new = target_pose.header.stamp.sec*1000.0 + target_pose.header.stamp.nsec/1000000.0;
    		dt = time_new - time_old;
   
    		v1 = sqrt(pow(x - x_old,2)+pow(y - y_old,2))/double(dt)*1000.0;
   
    		time_old = time_new;
    
        	x_old = x;
   
		y_old = y;
   
    		v1i = v1i + (vd.at(goal_flag - 1) - v1)*double(dt)/1000.0;
    
    		ROS_INFO("%f,%f,%f,%f,%f",go_kp.at(goal_flag -1),vd.at(goal_flag - 1),go_ki.at(goal_flag - 1),v1,v1i);
    
    		u1 =go_kp.at(goal_flag -1)*(vd.at(goal_flag - 1) - v1) + go_ki.at(goal_flag - 1)*v1i;

    		ROS_INFO("%f,%f",go_kp.at(goal_flag -1),go_ki.at(goal_flag - 1));
    		e = (yr.at(goal_flag - 1) - y); 
    

    		if(this->x_or_y.at(goal_flag) == (int)go_status::y_axis_go )   
    			{e = (xr.at(goal_flag - 1) - x);}
   		ei = ei + e*double(dt)/1000.0;   
                ed = (e - e0.at(goal_flag -1))/double(dt)*1000.0;

                u2 = kp.at(goal_flag -1)*e+ ki.at(goal_flag -1)*ei + kd.at(goal_flag -1)*ed;    //旋回方向速度入力(PID control) 

                ROS_INFO("%f,%f",u2,u1);

                e0.at(goal_flag -1) = e;

                if(this->x_or_y.at(goal_flag) == (int)go_status::x_axis_go){      
                	if(((x > xd.at(goal_flag - 1)) && (x_begin < xd.at(goal_flag - 1))) || ((x < xd.at(goal_flag - 1)) && (x_begin > xd.at(goal_flag - 1)))){
                		u1 = 0;
        			u2 = 0;
        			goal_flag++;
       			}
        
    		}
    		else{
        		if(((y > yd.at(goal_flag - 1)) && (y_begin < yd.at(goal_flag - 1))) || ((y < yd.at(goal_flag - 1)) && (y_begin > yd.at(goal_flag - 1)))){
        			u1 = 0;
        			u2 = 0;
        			goal_flag++;
        		}
    		}


    		if(this->x_or_y.at(goal_flag) == (int)go_status::x_axis_go){
        		twist.linear.x = u1;
        		twist.linear.y = 0;
        	}
    		else{
        		twist.linear.x = 0;
        		twist.linear.y = u1;
		}
		twist.angular.z = u2;
    		ROS_INFO("x,y,yr,v1,dt,%f,%f, %lf,%f,%lf",x,y,yr.at(goal_flag - 1),v1,dt);
    		if((u1 > 1.5) || (u1 < -1.5) || ((int)x_or_y.size() < goal_flag)){       
        		this->reset();
    		}
    		_vel_pub.publish(twist);}
	}

}

void Sp_auto_node::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
static bool ButtonA_b = false;
static bool ButtonLB_b = false;
static bool ButtonRB_b = false;


bool _ButtonA_b = joy->buttons[ButtonA];
bool _ButtonLB_b = joy->buttons[ButtonLB];
bool _ButtonRB_b = joy->buttons[ButtonRB];

if (_ButtonA_b && _ButtonLB_b){
    ROS_INFO("auto set");
    this->go_x_set(2.460,-0.1,-0.2,5.0,0.0,8.0,0.3,0.5);
    this->go_y_set(-2.620,-0.1,2.460,5.0,0.0,8.0,0.3,0.5);
    goal_flag++;}
         
ButtonA_b = _ButtonA_b;
ButtonLB_b = _ButtonLB_b;
ButtonRB_b = _ButtonRB_b;



}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "spring_20_semiauto");

    Sp_auto_node sp_auto_node;

    ros::spin();
}
