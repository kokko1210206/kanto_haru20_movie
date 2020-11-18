/*
 * kari.cpp
 *
 *  Created on: Aug 19, 2019
 *      Author: shun
 */

/*
 * haru20_b_joy_main_semiauto.cpp
 *
 *  Created on: Dec 15, 2019
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

#include "Coordinates_haru.hpp"

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

enum class ControllerStatus : uint16_t
{
    init        = 0x0000,
    shutdown    = 0x0001,

    standby     = 0x0010, // standby at start zone
    sz_to_fts,            // moving from SZ to FTS                ::FTSとはFlont of Transport Shipの略
    fts_to_ts,            // moving from FTS to TS                ::手動の予定。
    pick_ts,              // pick TS                              ::手動の予定。
    ts_to_bts,            // moving from TS to BTS                ::手動の予定。 BTSはBack of TS
    bts_to_pz,            // moving from BTS to PZ                ::PZはPlanet Zone
    put_ts,               // putting TS                           ::手動の予定。
    pz_to_gz,             // moving from PZ to GZ                 ::GZはGoods Zone　
    pick_goods,           // pick the goods                       ::goodsは物資。
    gz_to_pz,             // moving from PZ to GZ                 ::GZはGoods Zone　
    put_goods,            // putting goods                        ::手動の予定。
};

enum class ControllerCommands : uint16_t
{
    shutdown, // shutdown

    standby, // stand-by at SZ

    grab_ts,
    release_ts,
    grab_goods,
    release_goods,
 // load_shagai,
    put_arm_back,
    adjust_arm,
    expand,
    contract,
    put_ts,
    put_goods,
 // shrink_cylinder,

    sz_to_fts,
    fts_to_ts,
    ts_to_bts,
    bts_to_pz,
    pz_to_gz,
    gz_to_pz,

    set_delay_250ms,
    set_delay_500ms,
    set_delay_1s,

    delay,

//  segno,
//  dal_segno,

    wait_next_pressed,
};

enum class OpMode : uint8_t
{
    def         = 0b000,
    full_op     = 0b001,

    move_test   = 0b100,
    pickup_test = 0b101,
    put_test  = 0b110,
    pickup_and_put_test = 0b111,
};

enum class ArmStatus : uint16_t
{
    shutdown    = 0x0000,
    reset       = 0x0001,
};

enum class ArmCommands : uint16_t
{
    shutdown_cmd        = 0b0000,
    reset_cmd           = 0b1001,

    grab_ts_cmd     = 0b0001,
    grab_goods_cmd     = 0b0010,
    put_cmd           = 0b1000,
    
};

enum class BaseStatus : uint16_t
{
    shutdown    = 0x0000,
    reset       = 0x0001,
    operational = 0x0010,
};

enum class BaseCommands : uint16_t
{
    shutdown_cmd    = 0x0000,
    reset_cmd       = 0x0001,
    operational_cmd = 0x0010,
};

enum class MotorCommands : uint8_t
{
    shutdown_cmd    = 0x0000,
    reset_cmd       = 0x0001,
    homing_cmd      = 0x0010,
};


class CrMain
{
public:
    CrMain(void);

private:
    void shutdownInputCallback(const std_msgs::Empty::ConstPtr& msg);
    void startInputCallback(const std_msgs::Empty::ConstPtr& msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void baseStatusCallback(const std_msgs::UInt16::ConstPtr &msg);
    void baseConfCallback(const std_msgs::UInt8::ConstPtr &msg);
    void motorStatusCallback(const std_msgs::UInt8::ConstPtr &msg);
    void goalReachedCallback(const std_msgs::Bool::ConstPtr &msg);
    void control_timer_callback(const ros::TimerEvent &event);

    ros::NodeHandle nh_;

    //int linear_, angular_;
    ros::Subscriber joy_sub;
    ros::Subscriber shutdown_input_sub;
    ros::Subscriber start_input_sub;

    ros::Publisher pick_sh_position_pub;
    ros::Publisher pick_sh_enable_pub;

    ros::Publisher pick_ba_r_position_pub;
    ros::Publisher pick_ba_r_enable_pub;
    
    ros::Publisher pick_ba_l_position_pub;
    ros::Publisher pick_ba_l_enable_pub;

    ros::Publisher move_vert_position_pub;
    ros::Publisher move_vert_enable_pub;

    ros::Publisher act_enable_pub0;
    ros::Publisher act_enable_pub1;
    ros::Publisher act_enable_pub2;

    std_msgs::Float32 arm_position_msg;
    std_msgs::Float32 pick_position_msg;
    std_msgs::UInt8 act_enable_msg;

    double steps_per_mm = 1;

    std::vector<double> pick_position = { 0, -0.065, 1.2, -1.2 };
    int pick_position_index = 0;

    int pick_count_r = 0;
    int pick_count_l = 0;
    int pick_count_ship = 0;

    std::vector<double> vertical_position = { 0, -3.5};
    int vertical_position_index = 0;
    //		{0, -40 * steps_per_mm;
    //static constexpr int lift_position_first = -40 * steps_per_mm;
    //static constexpr int lift_position_second = lift_position_first - (248 * steps_per_mm);
    //static constexpr int lift_position_third = lift_position_second - (248 * steps_per_mm);

    bool _shutdown = true;

    static int ButtonX;
    static int ButtonA;
    static int ButtonB;
    static int ButtonY;
    static int ButtonLB;
    static int ButtonRB;
    static int ButtonLeftThumb;
    static int ButtonRightThumb;
    static int ButtonBack;
    static int ButtonStart;
    static int AxisLeftTrigger;
    static int AxisRightTrigger;
    static int AxisDPadX;
    static int AxisDPadY;
};

int CrMain::ButtonX = 0;
int CrMain::ButtonA = 1;
int CrMain::ButtonB = 2;
int CrMain::ButtonY = 3;
int CrMain::ButtonLB = 4;
int CrMain::ButtonRB = 5;
int CrMain::ButtonLeftThumb = 6;
int CrMain::ButtonRightThumb = 7;
int CrMain::ButtonBack = 8;
int CrMain::ButtonStart = 9;

int CrMain::AxisDPadX = 0;
int CrMain::AxisDPadY = 1;
CrMain::CrMain(void)
{
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &CrMain::joyCallback, this);
    shutdown_input_sub = nh_.subscribe<std_msgs::Empty>("shutdown_input", 10, &CrMain::shutdownInputCallback, this);
    start_input_sub = nh_.subscribe<std_msgs::Empty>("start_input", 10, &CrMain::startInputCallback, this);

    this->move_vert_position_pub = nh_.advertise<std_msgs::Float32>("pick/move_vert_cmd_pos", 1);
    this->move_vert_enable_pub = nh_.advertise<std_msgs::UInt8>("pick/move_vert_cmd", 1);
    this->pick_ba_r_position_pub = nh_.advertise<std_msgs::Float32>("pick/motorpc_ba_r_cmd_pos", 1);
    this->pick_ba_r_enable_pub = nh_.advertise<std_msgs::UInt8>("pick/motorpc_ba_r_cmd", 1);
    this->pick_ba_l_position_pub = nh_.advertise<std_msgs::Float32>("pick/motorpc_ba_l_cmd_pos", 1);
    this->pick_ba_l_enable_pub = nh_.advertise<std_msgs::UInt8>("pick/motorpc_ba_l_cmd", 1);
    this->pick_sh_position_pub = nh_.advertise<std_msgs::Float32>("pick/motorpc_sh_cmd_pos", 1);
    this->pick_sh_enable_pub = nh_.advertise<std_msgs::UInt8>("pick/motorpc_sh_cmd", 1);
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
        pos *= (steps_per_mm);
    }

    ROS_INFO("pick_pos: %f, %f, %f, %f, %f", this->pick_position[0], this->pick_position[1], this->pick_position[2],
            this->pick_position[3], this->pick_position[4]);

    nh_.getParam("ButtonX", ButtonX);
    nh_.getParam("ButtonA", ButtonA);
    nh_.getParam("ButtonB", ButtonB);
    nh_.getParam("ButtonY", ButtonY);
    nh_.getParam("ButtonLB", ButtonLB);
    nh_.getParam("ButtonRB", ButtonRB);
    nh_.getParam("ButtonLeftThumb", ButtonLeftThumb);
    nh_.getParam("ButtonRightThumb", ButtonRightThumb);
    nh_.getParam("ButtonBack", ButtonBack);
    nh_.getParam("ButtonStart", ButtonStart);
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
    pick_sh_enable_pub.publish(act_enable_msg);
    pick_ba_r_enable_pub.publish(act_enable_msg); 
    pick_ba_l_enable_pub.publish(act_enable_msg);
    move_vert_enable_pub.publish(act_enable_msg);
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
    static bool last_7 = false;
    static bool last_8 = false;
    static bool last_9 = false;
    static bool last_10 = false;
    static bool last_11 = false;
    //static int last_dpadXCmd = 0;

    bool _1 = joy->buttons[ButtonX];
    bool _2 = joy->buttons[ButtonA];
    bool _3 = joy->buttons[ButtonB];
    bool _4 = joy->buttons[ButtonY];
    bool _5 = joy->buttons[ButtonLB];
    bool _6 = joy->buttons[ButtonRB];
    bool _7 = joy->buttons[ButtonLeftThumb];
    bool _8 = joy->buttons[ButtonRightThumb];
    bool _9 = joy->buttons[ButtonBack];
    bool _10 = joy->buttons[ButtonStart];
    bool _11 = joy->axes[AxisDPadY];
    if (_10 && !last_10)
    {
        ROS_INFO("starting.");

        act_enable_msg.data = 1;
        act_enable_pub0.publish(act_enable_msg);
        act_enable_pub1.publish(act_enable_msg);
        act_enable_pub2.publish(act_enable_msg);
        pick_sh_enable_pub.publish(act_enable_msg);
        pick_ba_r_enable_pub.publish(act_enable_msg); 
        pick_ba_l_enable_pub.publish(act_enable_msg); 
        move_vert_enable_pub.publish(act_enable_msg);

        this->_shutdown = 0;  
    }


    if (_9 && !last_9)
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
        pick_sh_enable_pub.publish(act_enable_msg);
        pick_ba_r_enable_pub.publish(act_enable_msg); 
        pick_ba_l_enable_pub.publish(act_enable_msg);
        move_vert_enable_pub.publish(act_enable_msg);
    }

    if (!this->_shutdown)
    {
        
        
        if (_3 && !last_3)
        {
            // pick and put ball (right)
            pick_count_r++;
            if (pick_count_r % 2 == 0){
            pick_position_index = 0;
            pick_position_msg.data = pick_position[pick_position_index];
            pick_ba_r_position_pub.publish(pick_position_msg);
            ROS_INFO("put　right start");  }
           
            else{
            pick_position_index = 2;
            pick_position_msg.data = pick_position[pick_position_index];
            pick_ba_r_position_pub.publish(pick_position_msg);
            ROS_INFO("pick　right start");}

        }
        else if (_4 && !last_4)
        {
            // pick and put the ship

            pick_count_r++;
            if (pick_count_r % 2 == 0){
            pick_position_index = 0;
            pick_position_msg.data = pick_position[pick_position_index];
            pick_sh_position_pub.publish(pick_position_msg);
            ROS_INFO("put　ship start");  }
           
            else{
            pick_position_index = 1;
            pick_position_msg.data = pick_position[pick_position_index];
            pick_sh_position_pub.publish(pick_position_msg);
            ROS_INFO("pick ship start");}
            
        }
        else if (_1 && !last_1)
        {
            // pick and put ball (left)
            pick_count_l++;
            if (pick_count_l % 2 == 0){
            pick_position_index = 0;
            pick_position_msg.data = pick_position[pick_position_index];
            pick_ba_l_position_pub.publish(pick_position_msg);
            ROS_INFO("put left start");  }
           
            else{
            pick_position_index = 3;
            pick_position_msg.data = pick_position[pick_position_index];
            pick_ba_l_position_pub.publish(pick_position_msg);
            ROS_INFO("pick left start");}
        }
        
        
        else if (_11 && !last_11)
        {
            // arm moves vertically up 
            if (joy->axes[AxisDPadY] == 1){
            vertical_position_index = 1;
            arm_position_msg.data = vertical_position[vertical_position_index];
            move_vert_position_pub.publish(arm_position_msg);
            ROS_INFO("arm moves vertically　up");}
            else{
            vertical_position_index = 0;
            arm_position_msg.data = vertical_position[vertical_position_index];
            move_vert_position_pub.publish(arm_position_msg);
            ROS_INFO("arm moves vertically　down");
            }
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
    last_9 = _9;
    last_10 = _10;
    last_11 = _11;
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "haru20_movie_main");

    CrMain *crMain = new CrMain();
    ROS_INFO("haru20_movie_main node has started.");

    ros::spin();
    ROS_INFO("haru20_movie_main node has been terminated.");
}

