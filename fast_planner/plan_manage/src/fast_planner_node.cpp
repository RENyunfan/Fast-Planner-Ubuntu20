/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/kino_replan_fsm.h>
#include <plan_manage/topo_replan_fsm.h>

#include <plan_manage/backward.hpp>
#include "quadrotor_msgs/PositionCommand.h"

namespace backward {
    backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char **argv) {
    ros::init(argc, argv, "fast_planner_node");
    ros::NodeHandle nh("~");

    int planner;
    nh.param("planner_node/planner", planner, -1);
    bool benchmark_en{false};
    nh.param("/benchmark_en", benchmark_en, false);

    TopoReplanFSM topo_replan;
    KinoReplanFSM kino_replan;

    if (planner == 1) {
        kino_replan.init(nh);
    } else if (planner == 2) {
        topo_replan.init(nh);
    }

    if (benchmark_en) {
        cout << " -- [FSM] benchmark_en" << endl;
        cout << " -- [FSM] benchmark_en" << endl;
        cout << " -- [FSM] benchmark_en" << endl;

        ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 1);
        quadrotor_msgs::PositionCommand cmd;

        TopoReplanFSM topo_replan;
        KinoReplanFSM kino_replan;

        ros::Duration(0.2).sleep();

        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";
        cmd.position.x = 0.0;
        cmd.position.y = -50.0;
        cmd.position.z = 1.5;
        cmd.velocity.x = 0.0;
        cmd.velocity.y = 0.0;
        cmd.velocity.z = 0.0;
        cmd.acceleration.x = 0.0;
        cmd.acceleration.y = 0.0;
        cmd.acceleration.z = 0.0;
        cmd.yaw = 1.5741;
        cmd.yaw_dot = 0.0;
        int cnt = 10;
        while (cnt--) {
            cmd_pub.publish(cmd);
            ros::Duration(0.1).sleep();
        }
    }


    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
