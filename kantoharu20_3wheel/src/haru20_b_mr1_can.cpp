/*
 * mr1_can.cpp
 *
 *  Created on: Feb 27, 2019
 *      Author: yuto
 */

/*
 * mr1_can_natu.cpp
 *
 *  Created on: Aug 19, 2019
 *      Author: shun
 */

/*
 * kantoharu20_mr1_can.cpp
 *
 *  Created on: Nov 30, 2019
 *      Author: shun
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <boost/array.hpp>

#include <can_msgs/CanFrame.h>

#define CAN_MTU 8

template<typename T>
union _Encapsulator
{
    T data;
    uint64_t i;
};

template <typename T>
void can_unpack(const boost::array<uint8_t, CAN_MTU> &buf, T &data)
{
    _Encapsulator<T> _e;

    for(int i = 0; i < sizeof(T); i++)
    {
        _e.i = (_e.i << 8) | (uint64_t)(buf[i]);
    }

    data = _e.data;
}

template<typename T>
void can_pack(boost::array<uint8_t, CAN_MTU> &buf, const T data)
{
    _Encapsulator<T> _e;
    _e.data = data;

    for(int i = sizeof(T); i > 0;)
    {
        i--;
        buf[i] = _e.i & 0xff;
        _e.i >>= 8;
    }
}

class Mr1CanNode
{
public:
    Mr1CanNode(void);

private:

    void baseCmdCallback(const std_msgs::UInt16::ConstPtr& msg);

    void ft0motorCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
    void ft0motorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg); 
    void ft1motorCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
    void ft1motorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg);    
    void ft2motorCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
    void ft2motorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg);     
  
    void thmotorCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
    void thmotorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg);   
    void pcmotorCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
    void pcmotorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg);   

    void canRxCallback(const can_msgs::CanFrame::ConstPtr &msg);

    template<typename T>
    void sendData(const uint16_t id, const T data);

    ros::NodeHandle _nh;
    ros::Publisher  _can_tx_pub;
    ros::Subscriber _can_rx_sub;

    ros::Publisher  _base_status_pub;
    ros::Publisher  _base_conf_pub;

    ros::Publisher  _ft_0_motor_status_pub;
    ros::Subscriber _ft_0_motor_cmd_sub;
    ros::Subscriber _ft_0_motor_cmd_pos_sub;

    ros::Publisher  _ft_1_motor_status_pub;
    ros::Subscriber _ft_1_motor_cmd_sub;
    ros::Subscriber _ft_1_motor_cmd_pos_sub;

    ros::Publisher  _ft_2_motor_status_pub;
    ros::Subscriber _ft_2_motor_cmd_sub;
    ros::Subscriber _ft_2_motor_cmd_pos_sub;

    ros::Publisher  _th_motor_status_pub;
    ros::Subscriber _th_motor_cmd_sub;
    ros::Subscriber _th_motor_cmd_pos_sub;

    ros::Publisher  _pc_motor_status_pub;
    ros::Subscriber _pc_motor_cmd_sub;
    ros::Subscriber _pc_motor_cmd_pos_sub;

    static constexpr uint16_t id_baseStatus             = 0x200;
    static constexpr uint16_t id_baseOdomX              = 0x205;
    static constexpr uint16_t id_baseOdomY              = 0x206;
    static constexpr uint16_t id_baseOdomYaw            = 0x207;
    static constexpr uint16_t id_baseConf               = 0x208;

    static constexpr uint16_t id_ft_0_motor_cmd         = 0x4b8;
    static constexpr uint16_t id_ft_0_motor_cmd_pos     = (id_ft_0_motor_cmd + 1);
    static constexpr uint16_t id_ft_0_motor_status      = (id_ft_0_motor_cmd + 2);

    static constexpr uint16_t id_ft_1_motor_cmd         = 0x4d0;
    static constexpr uint16_t id_ft_1_motor_cmd_pos     = (id_ft_1_motor_cmd + 1);
    static constexpr uint16_t id_ft_1_motor_status      = (id_ft_1_motor_cmd + 2);

    static constexpr uint16_t id_ft_2_motor_cmd         = 0x4d4;
    static constexpr uint16_t id_ft_2_motor_cmd_pos     = (id_ft_2_motor_cmd + 1);
    static constexpr uint16_t id_ft_2_motor_status      = (id_ft_2_motor_cmd + 2);

    static constexpr uint16_t id_th_motor_cmd           = 0x4f0;
    static constexpr uint16_t id_th_motor_cmd_pos       = (id_th_motor_cmd + 1);
    static constexpr uint16_t id_th_motor_status        = (id_th_motor_cmd + 2);

    static constexpr uint16_t id_pc_motor_cmd           = 0x4f4;
    static constexpr uint16_t id_pc_motor_cmd_pos       = (id_pc_motor_cmd + 1);
    static constexpr uint16_t id_pc_motor_status        = (id_pc_motor_cmd + 2);


};

Mr1CanNode::Mr1CanNode(void)
{
    _can_tx_pub				    = _nh.advertise<can_msgs::CanFrame>("can_tx", 10);
    _can_rx_sub				    = _nh.subscribe<can_msgs::CanFrame>("can_rx", 10, &Mr1CanNode::canRxCallback, this);

    _base_status_pub		    	    = _nh.advertise<std_msgs::UInt16>("base/status", 10);
    _base_conf_pub			    = _nh.advertise<std_msgs::UInt8>("base/conf", 10);

    _ft_0_motor_status_pub      	    = _nh.advertise<std_msgs::UInt8>("motor_status", 10);
    _ft_0_motor_cmd_sub	        	    = _nh.subscribe<std_msgs::UInt8>("base/motor0_cmd", 10, &Mr1CanNode::ft0motorCmdCallback, this);
    _ft_0_motor_cmd_pos_sub	    	    = _nh.subscribe<std_msgs::Float32>("base/motor0_cmd_vel", 10, &Mr1CanNode::ft0motorCmdPosCallback, this);

    _ft_1_motor_status_pub      	    = _nh.advertise<std_msgs::UInt8>("motor_status", 10);
    _ft_1_motor_cmd_sub	          	    = _nh.subscribe<std_msgs::UInt8>("base/motor1_cmd", 10, &Mr1CanNode::ft1motorCmdCallback, this);
    _ft_1_motor_cmd_pos_sub	    	    = _nh.subscribe<std_msgs::Float32>("base/motor1_cmd_vel", 10, &Mr1CanNode::ft1motorCmdPosCallback, this);

    _ft_2_motor_status_pub      	    = _nh.advertise<std_msgs::UInt8>("motor_status", 10);
    _ft_2_motor_cmd_sub	        	    = _nh.subscribe<std_msgs::UInt8>("base/motor2_cmd", 10, &Mr1CanNode::ft2motorCmdCallback, this);
    _ft_2_motor_cmd_pos_sub	    	    = _nh.subscribe<std_msgs::Float32>("base/motor2_cmd_vel", 10, &Mr1CanNode::ft2motorCmdPosCallback, this);

    _th_motor_status_pub      		    = _nh.advertise<std_msgs::UInt8>("motor_status", 10);
    _th_motor_cmd_sub	        	    = _nh.subscribe<std_msgs::UInt8>("throw/motorth_cmd", 10, &Mr1CanNode::thmotorCmdCallback, this);
    _th_motor_cmd_pos_sub	    	    = _nh.subscribe<std_msgs::Float32>("throw/motorth_cmd_pos", 10, &Mr1CanNode::thmotorCmdPosCallback, this);

    _pc_motor_status_pub      		    = _nh.advertise<std_msgs::UInt8>("motor_status", 10);
    _pc_motor_cmd_sub	        	    = _nh.subscribe<std_msgs::UInt8>("pick/motorpc_cmd", 10, &Mr1CanNode::pcmotorCmdCallback, this);
    _pc_motor_cmd_pos_sub	   	    = _nh.subscribe<std_msgs::Float32>("pick/motorpc_cmd_pos", 10, &Mr1CanNode::pcmotorCmdPosCallback, this);

}


void Mr1CanNode::ft0motorCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    this->sendData(id_ft_0_motor_cmd, msg->data);
}

void Mr1CanNode::ft0motorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->sendData(id_ft_0_motor_cmd_pos, msg->data);
}

void Mr1CanNode::ft1motorCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    this->sendData(id_ft_1_motor_cmd, msg->data);
}

void Mr1CanNode::ft1motorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->sendData(id_ft_1_motor_cmd_pos, msg->data);
}

void Mr1CanNode::ft2motorCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    this->sendData(id_ft_2_motor_cmd, msg->data);
}

void Mr1CanNode::ft2motorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->sendData(id_ft_2_motor_cmd_pos, msg->data);
}

void Mr1CanNode::thmotorCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    this->sendData(id_th_motor_cmd, msg->data);
}

void Mr1CanNode::thmotorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->sendData(id_th_motor_cmd_pos, msg->data);
}

void Mr1CanNode::pcmotorCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    this->sendData(id_pc_motor_cmd, msg->data);
}

void Mr1CanNode::pcmotorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->sendData(id_pc_motor_cmd_pos, msg->data);
}

void Mr1CanNode::canRxCallback(const can_msgs::CanFrame::ConstPtr &msg)
{
    std_msgs::UInt16 _base_status_msg;
    std_msgs::UInt8 _base_conf_msg;
    std_msgs::UInt8 _ft_0_motor_status_msg;
    std_msgs::UInt8 _ft_1_motor_status_msg;
    std_msgs::UInt8 _ft_2_motor_status_msg;
    std_msgs::UInt8 _th_motor_status_msg;
    std_msgs::UInt8 _pc_motor_status_msg;

    switch(msg->id)
    {

        case id_baseStatus:
            can_unpack(msg->data, _base_status_msg.data);
            _base_status_pub.publish(_base_status_msg);
            break;

        case id_baseConf:
            can_unpack(msg->data, _base_conf_msg.data);
            _base_conf_pub.publish(_base_conf_msg);
            break;

        case id_ft_0_motor_status:
            can_unpack(msg->data, _ft_0_motor_status_msg.data);
            _ft_0_motor_status_pub.publish(_ft_0_motor_status_msg);
            break;

        case id_ft_1_motor_status:
            can_unpack(msg->data, _ft_1_motor_status_msg.data);
            _ft_1_motor_status_pub.publish(_ft_1_motor_status_msg);
            break;

        case id_ft_2_motor_status:
            can_unpack(msg->data, _ft_2_motor_status_msg.data);
            _ft_2_motor_status_pub.publish(_ft_2_motor_status_msg);
            break;

        case id_th_motor_status:
            can_unpack(msg->data, _th_motor_status_msg.data);
            _th_motor_status_pub.publish(_th_motor_status_msg);
            break;

        case id_pc_motor_status:
            can_unpack(msg->data, _pc_motor_status_msg.data);
            _pc_motor_status_pub.publish(_pc_motor_status_msg);
            break;

        default:
            break;
    }
}

template<typename T>
void Mr1CanNode::sendData(const uint16_t id, const T data)
{
    can_msgs::CanFrame frame;
    frame.id = id;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;

    frame.dlc = sizeof(T);

    can_pack<T>(frame.data, data);

    _can_tx_pub.publish(frame);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mr1_can");
    ROS_INFO("mr1_can node has started.");

    Mr1CanNode *mr1CanNode = new Mr1CanNode();

    ros::spin();
}

