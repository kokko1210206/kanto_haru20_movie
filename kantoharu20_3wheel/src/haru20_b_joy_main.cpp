/*
 * kari.cpp
 *
 *  Created on: Aug 19, 2019
 *      Author: shun
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <string>

enum class CarrierStatus
    : uint16_t
    {
        shutdown = 0x0000,
    reset = 0x0001,

/*
 operational			= 0x0002,

 chuck0_chucked		= 0x0010,
 chuck1_chucked		= 0x0020,
 chuck2_chucked		= 0x0040,
 chuck3_chucked		= 0x0080,
 */
};

enum class CarrierCommands
    : uint16_t
    {
        shutdown_cmd = 0x0000,
    reset_cmd = 0x0001,
    /*
     operational			= 0x0002,
     */

    chuck_cmd = 0x0100,
    unchuck_cmd = 0x0200,

    chuck0 = 0x0010,
    chuck1 = 0x0020,
    chuck2 = 0x0040,
    chuck3 = 0x0080,
};

class CrMain
{
public:
    CrMain(void);

private:
    void shutdownInputCallback(const std_msgs::Empty::ConstPtr& msg);
    void startInputCallback(const std_msgs::Empty::ConstPtr& msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    //int linear_, angular_;
    ros::Subscriber joy_sub;
    ros::Subscriber shutdown_input_sub;
    ros::Subscriber start_input_sub;

    ros::Publisher pick_position_pub;
    ros::Publisher pick_enable_pub;

    ros::Publisher throw_position_pub;
    ros::Publisher throw_enable_pub;

    ros::Publisher act_enable_pub0;
    ros::Publisher act_enable_pub1;
    ros::Publisher act_enable_pub2;

    std_msgs::Float32 pick_position_msg;
    std_msgs::Float32 throw_position_msg;
    std_msgs::UInt8 act_enable_msg;

    double steps_per_mm = 1;

    std::vector<double> pick_position = { 0, 3.7, 0, 0, 0 };
    int pick_position_index = 0;

    std::vector<double> throw_position = { 0, 9.1, 13.2, 0, 0 };
    int throw_position_index = 0;
    //		{0, -40 * steps_per_mm;
    //static constexpr int lift_position_first = -40 * steps_per_mm;
    //static constexpr int lift_position_second = lift_position_first - (248 * steps_per_mm);
    //static constexpr int lift_position_third = lift_position_second - (248 * steps_per_mm);

    bool _shutdown = true;

    static int Button1;
    static int Button2;
    static int Button3;
    static int Button4;
    static int Button5;
    static int Button6;
    static int Button11;
    static int Button12;
    static int Button7;
    static int Button8;

    static int AxisDPadX;
    static int AxisDPadY;
};

int CrMain::Button1 = 0;
int CrMain::Button2 = 1;
int CrMain::Button3 = 2;
int CrMain::Button4 = 3;
int CrMain::Button5 = 4;
int CrMain::Button6 = 5;
int CrMain::Button7 = 6;
int CrMain::Button8 = 7;
int CrMain::Button11 = 10;
int CrMain::Button12 = 11;

int CrMain::AxisDPadX = 6;
int CrMain::AxisDPadY = 7;

CrMain::CrMain(void)
{
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &CrMain::joyCallback, this);
    shutdown_input_sub = nh_.subscribe<std_msgs::Empty>("shutdown_input", 10, &CrMain::shutdownInputCallback, this);
    start_input_sub = nh_.subscribe<std_msgs::Empty>("start_input", 10, &CrMain::startInputCallback, this);

    this->throw_position_pub = nh_.advertise<std_msgs::Float32>("throw/motorth_cmd_pos", 1);
    this->throw_enable_pub = nh_.advertise<std_msgs::UInt8>("throw/motorth_cmd", 1);
    this->pick_position_pub = nh_.advertise<std_msgs::Float32>("pick/motorpc_cmd_pos", 1);
    this->pick_enable_pub = nh_.advertise<std_msgs::UInt8>("pick/motorpc_cmd", 1);
    this->act_enable_pub0 = nh_.advertise<std_msgs::UInt8>("base/motor0_cmd", 1);
    this->act_enable_pub1 = nh_.advertise<std_msgs::UInt8>("base/motor1_cmd", 1);
    this->act_enable_pub2 = nh_.advertise<std_msgs::UInt8>("base/motor2_cmd", 1);
    //this->hand_unchuck_thres_pub = nh_.advertise<std_msgs::UInt16>("hand/unchuck_thres", 1);

    auto nh_priv = ros::NodeHandle("~");

    nh_priv.getParam("lift_step_per_mm", this->steps_per_mm);

    std::vector<double> tmp;
    nh_priv.getParam("pick_position", tmp);
    if (tmp.size() == 5)
    {
        this->pick_position = tmp;
    }

    for (double& pos : this->pick_position)
    {
        pos *= (-steps_per_mm);
    }

    ROS_INFO("pick_pos: %f, %f, %f, %f, %f", this->pick_position[0], this->pick_position[1], this->pick_position[2],
            this->pick_position[3], this->pick_position[4]);

    nh_.getParam("Button1", Button1);
    nh_.getParam("Button2", Button2);
    nh_.getParam("Button3", Button3);
    nh_.getParam("Button4", Button4);
    nh_.getParam("Button5", Button5);
    nh_.getParam("Button6", Button6);
    nh_.getParam("Button11", Button11);
    nh_.getParam("Button12", Button12);
    nh_.getParam("Button7", Button7);
    nh_.getParam("Button8", Button8);

    nh_.getParam("AxisDPadX", AxisDPadX);
    nh_.getParam("AxisDPadY", AxisDPadY);
}

void CrMain::shutdownInputCallback(const std_msgs::Empty::ConstPtr& msg)
{
    if (!this->_shutdown)
    {
//        this->_shutdown = 1;

//        ROS_INFO("aborting.");
    }

    // reset this:
    // this->CurrentCommandIndex = -1;
    pick_position_index = 0;
}

void CrMain::startInputCallback(const std_msgs::Empty::ConstPtr& msg)
{
    // bring the robot back operational

    ROS_INFO("starting.");

    act_enable_msg.data = 1;
    act_enable_pub0.publish(act_enable_msg);
    act_enable_pub1.publish(act_enable_msg);
    act_enable_pub2.publish(act_enable_msg);
    pick_enable_pub.publish(act_enable_msg);
    throw_enable_pub.publish(act_enable_msg);

    this->_shutdown = 0;
}

void CrMain::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    static bool last_1 = false;
    static bool last_2 = false;
    static bool last_3 = false;
    static bool last_4 = false;
    static bool last_5 = false;
    static bool last_6 = false;
    static bool last_11 = false;
    static bool last_12 = false;
    static bool last_7 = false;
    static bool last_8 = false;
    //static int last_dpadXCmd = 0;

    bool _1 = joy->buttons[Button1];
    bool _2 = joy->buttons[Button2];
    bool _3 = joy->buttons[Button3];
    bool _4 = joy->buttons[Button4];
    bool _5 = joy->buttons[Button5];
    bool _6 = joy->buttons[Button6];
    bool _11 = joy->buttons[Button11];
    bool _12 = joy->buttons[Button12];
    bool _7 = joy->buttons[Button7];
    bool _8 = joy->buttons[Button8];

    if (_12 && !last_12)
    {
        ROS_INFO("starting.");

        act_enable_msg.data = 1;
        act_enable_pub0.publish(act_enable_msg);
        act_enable_pub1.publish(act_enable_msg);
        act_enable_pub2.publish(act_enable_msg);
        pick_enable_pub.publish(act_enable_msg);
        throw_enable_pub.publish(act_enable_msg);

        this->_shutdown = 0;  
    }


    if (_11 && !last_11)
    {
        if (!this->_shutdown)
        {
            this->_shutdown = 1;

            ROS_INFO("aborting.");
        }
        
        act_enable_msg.data = 0;
        act_enable_pub0.publish(act_enable_msg);
        act_enable_pub1.publish(act_enable_msg);
        act_enable_pub2.publish(act_enable_msg);
        pick_enable_pub.publish(act_enable_msg);
        throw_enable_pub.publish(act_enable_msg);
    }

    if (!this->_shutdown)
    {
        if (_7 && !last_7)
        {
            // receive  the throw
            throw_position_index = 1;
            throw_position_msg.data = throw_position[throw_position_index];
            throw_position_pub.publish(throw_position_msg);

            ROS_INFO("th receive");
        }
        else if (_8 && !last_8)
        {
            //  the throw
            throw_position_index = 2;
            throw_position_msg.data = throw_position[throw_position_index];
            throw_position_pub.publish(throw_position_msg);

            ROS_INFO("th throw");
        }
       else if (_3 && !last_3)
        {
            // stop the pick
            pick_position_index = 0;
            pick_position_msg.data = pick_position[pick_position_index];
            pick_position_pub.publish(pick_position_msg);

            ROS_INFO("pc stop");

        }
        else if (_4 && !last_4)
        {
            // stop the throw
            throw_position_index = 0;
            throw_position_msg.data = throw_position[throw_position_index];
            throw_position_pub.publish(throw_position_msg);

            ROS_INFO("th stop");
        }
        else if (_1 && !last_1)
        {
            // start the pick
            act_enable_msg.data = 0;
            pick_enable_pub.publish(act_enable_msg);

            ROS_INFO("pc start");
        }
        else if (_2 && !last_2)
        {
            // shutdown the pick
            act_enable_msg.data = 0;
            pick_enable_pub.publish(act_enable_msg);

            ROS_INFO("pc shutdown");
        }
        else if (_5 && !last_5)
        {
            // set the pick
            pick_position_index = 1;
            pick_position_msg.data = pick_position[pick_position_index];
            pick_position_pub.publish(pick_position_msg);

            ROS_INFO("pc set");
        }
        else if (_6 && !last_6)
        {
            // serve the pick
            pick_position_index = 2;
            pick_position_msg.data = pick_position[pick_position_index];
            pick_position_pub.publish(pick_position_msg);

            ROS_INFO("pc serve");
        }
    }


    last_1 = _1;
    last_2 = _2;
    last_3 = _3;
    last_4 = _4;
    last_5 = _5;
    last_6 = _6;
    last_7 = _7;
    last_8 = _8;
    last_11 = _11;
    last_12 = _12;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "natu19_b_joy_main");

    CrMain *crMain = new CrMain();
    ROS_INFO("natu19_b_joy_main node has started.");

    ros::spin();
    ROS_INFO("natu19_b_joy_main node has been terminated.");
}

