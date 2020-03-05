/*
* Coordinates.hpp
*
*  Created on: Feb 27, 2019
*      Author: yuto
*/

#pragma once

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

class Coordinates
{
private:
    nav_msgs::Path _sz_to_pp;
    nav_msgs::Path _pp_to_pp1;
    nav_msgs::Path _pp1_to_tp;
    nav_msgs::Path _tp_to_pp2;
    nav_msgs::Path _pp2_to_tp;
    nav_msgs::Path _tp_to_pp3;
    nav_msgs::Path _pp3_to_tp;
    nav_msgs::Path _tp_to_sz;

    std::vector<std::vector<double>> _startzone_to_passPoint{
        {0.550, 9.550, -M_PI/2},
        {2.050, 8.200, -M_PI/2},
        {1.275, 7.300, -M_PI/2},
        {0.550, 6.700, -M_PI/2},
        {1.275, 5.800, -M_PI/2},
        {2.050, 5.200, -M_PI/2},
        {1.350, 4.200, -M_PI/2},
        {1.275, 2.800, -M_PI/2},
        {1.275, 1.550, -M_PI/2},
        {1.750, 1.250, -M_PI/4},
        {2.500, 1.350, 0.0},
        {4.500, 1.450, 0.0},
        {5.500, 1.550, 0.0}
    };
    
    std::vector<std::vector<double>> _passPoint_to_pickupPoint1{
        {5.500, 1.550, 0.0},
        {4.525, 1.300, -M_PI/4},
        {4.025, 1.050, -M_PI/2}
    };

    std::vector<std::vector<double>> _pickupPoint1_to_throwPoint{
        {5.500, 1.550, 0.0},
        {4.525, 1.450, -M_PI/4.0},
        {4.125, 1.350, -M_PI/3.0},
        {4.025, 2.550, -M_PI/2.0},
        {4.025, 3.550, -M_PI/1.7},
        {4.025, 5.450, -M_PI*3/4}
    };

    std::vector<std::vector<double>> _throwPoint_to_pickupPoint2{
        {4.025, 5.450, -M_PI*3/4},
        {4.025, 4.450, -M_PI/1.7},
        {4.025, 3.750, -M_PI/1.8},
        {4.025, 2.750, -M_PI/1.9},
        {4.025, 1.050, -M_PI/2.0}
    };

    std::vector<std::vector<double>> _pickupPoint2_to_throwPoint{
        {4.025, 1.050, -M_PI/2.0},
        {4.025, 2.550, -M_PI/1.7},
        {4.025, 3.550, -M_PI/1.6},
        {4.025, 4.650, -M_PI/1.4},
        {4.025, 5.450, -M_PI*3/4}
    };

    std::vector<std::vector<double>> _throwPoint_to_pickupPoint3{
        {4.025, 5.450, -M_PI*3/4},
        {4.025, 4.450, -M_PI/1.7},
        {4.025, 3.750, -M_PI/1.8},
        {4.025, 2.750, -M_PI/1.9},
        {4.025, 1.050, -M_PI/2.0}
    };

    std::vector<std::vector<double>> _pickupPoint3_to_throwPoint{
        {4.025, 1.050, -M_PI/2.0},
        {4.025, 2.550, -M_PI/1.7},
        {4.025, 3.550, -M_PI/1.6},
        {4.025, 4.650, -M_PI/1.4},
        {4.025, 5.450, -M_PI*3/4}
    };

    std::vector<std::vector<double>> _throwPoint_to_startzone{
        {4.025, 5.450, M_PI/4},
        {3.525, 3.450, 0.0},
        {3.525, 1.550, -M_PI/4},
        {1.275, 1.350, -M_PI/2},
        {1.275, 2.800, -M_PI/2},
        {1.275, 4.300, -M_PI/2},
        {0.550, 5.350, -M_PI/2},
        {0.550, 6.550, -M_PI/2},
        {0.550, 9.550, -M_PI/2}
    };

    static const Coordinates *instance;

public:
    Coordinates(void);

    static inline const Coordinates * const GetInstance(void)
    {
        return instance;
    }

    inline geometry_msgs::Pose get_sz(void) const
    {
        return this->_sz_to_pp.poses.front().pose;
    }

    inline nav_msgs::Path get_path_sz_to_pp(void) const
    {
        return this->_sz_to_pp;
    }
    inline nav_msgs::Path get_path_pp_to_pp1(void) const
    {
        return this->_pp_to_pp1;
    }
    inline nav_msgs::Path get_path_pp1_to_tp(void) const
    {
        return this->_pp1_to_tp;
    }
    inline nav_msgs::Path get_path_tp_to_pp2(void) const
    {
        return this->_tp_to_pp2;
    }
    inline nav_msgs::Path get_path_pp2_to_tp(void) const
    {
        return this->_pp2_to_tp;
    }
    inline nav_msgs::Path get_path_tp_to_pp3(void) const
    {
        return this->_tp_to_pp3;
    }
    inline nav_msgs::Path get_path_pp3_to_tp(void) const
    {
        return this->_pp3_to_tp;
    }
    inline nav_msgs::Path get_path_tp_to_sz(void) const
    {
        return this->_tp_to_sz;
    }
};

const Coordinates *Coordinates::instance = new Coordinates();

Coordinates::Coordinates(void)
{
    geometry_msgs::PoseStamped _pose;
    _pose.header.frame_id = "map";

    for (int i = 0; i < this->_startzone_to_passPoint.size(); i++)
    {
        _pose.pose.position.x = this->_startzone_to_passPoint[i][0];
        _pose.pose.position.y = this->_startzone_to_passPoint[i][1];
        _pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->_startzone_to_passPoint[i][2]);
        _sz_to_pp.poses.push_back(_pose);
    }

    for (int i = 0; i < this->_passPoint_to_pickupPoint1.size(); i++)
    {
        _pose.pose.position.x = this->_passPoint_to_pickupPoint1[i][0];
        _pose.pose.position.y = this->_passPoint_to_pickupPoint1[i][1];
        _pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->_passPoint_to_pickupPoint1[i][2]);
        _pp_to_pp1.poses.push_back(_pose);
    }

    for (int i = 0; i < this->_pickupPoint1_to_throwPoint.size(); i++)
    {
        _pose.pose.position.x = this->_pickupPoint1_to_throwPoint[i][0];
        _pose.pose.position.y = this->_pickupPoint1_to_throwPoint[i][1];
        _pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->_pickupPoint1_to_throwPoint[i][2]);
        _pp1_to_tp.poses.push_back(_pose);
    }

    for (int i = 0; i < this->_throwPoint_to_pickupPoint2.size(); i++)
    {
        _pose.pose.position.x = this->_throwPoint_to_pickupPoint2[i][0];
        _pose.pose.position.y = this->_throwPoint_to_pickupPoint2[i][1];
        _pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->_throwPoint_to_pickupPoint2[i][2]);
        _tp_to_pp2.poses.push_back(_pose);
    }

    for (int i = 0; i < this->_pickupPoint2_to_throwPoint.size(); i++)
    {
        _pose.pose.position.x = this->_pickupPoint2_to_throwPoint[i][0];
        _pose.pose.position.y = this->_pickupPoint2_to_throwPoint[i][1];
        _pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->_pickupPoint2_to_throwPoint[i][2]);
        _pp2_to_tp.poses.push_back(_pose);
    }

    for (int i = 0; i < this->_throwPoint_to_pickupPoint3.size(); i++)
    {
        _pose.pose.position.x = this->_throwPoint_to_pickupPoint3[i][0];
        _pose.pose.position.y = this->_throwPoint_to_pickupPoint3[i][1];
        _pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->_throwPoint_to_pickupPoint3[i][2]);
        _tp_to_pp3.poses.push_back(_pose);
    }

    for (int i = 0; i < this->_pickupPoint3_to_throwPoint.size(); i++)
    {
        _pose.pose.position.x = this->_pickupPoint3_to_throwPoint[i][0];
        _pose.pose.position.y = this->_pickupPoint3_to_throwPoint[i][1];
        _pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->_pickupPoint3_to_throwPoint[i][2]);
        _pp3_to_tp.poses.push_back(_pose);
    }

    for (int i = 0; i < this->_throwPoint_to_startzone.size(); i++)
    {
        _pose.pose.position.x = this->_throwPoint_to_startzone[i][0];
        _pose.pose.position.y = this->_throwPoint_to_startzone[i][1];
        _pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->_throwPoint_to_startzone[i][2]);
        _tp_to_sz.poses.push_back(_pose);
    }
}
