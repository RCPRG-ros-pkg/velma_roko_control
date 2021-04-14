/*
 Copyright (c) 2017, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/base/PortInterface.hpp>
#include "rtt_rosclock/rtt_rosclock.h"

#include "velma_core_cs_task_cs_msgs/CommandHead.h"
#include "velma_core_cs_task_cs_msgs/StatusHead.h"
#include "velma_core_cs_task_cs_msgs/VelmaHeadTrajectoryPoint.h"

namespace visual_servo_head_types {

class VisualServoComponent: public RTT::TaskContext {
public:
    explicit VisualServoComponent(const std::string &name)
        : RTT::TaskContext(name)
        , port_status_head_in_("st_head_INPORT")
        , port_cmd_head_out_("cmd_head_OUTPORT")
        , goal_pos_(1.0)    // For simple motion generation
    {
        this->ports()->addPort(port_status_head_in_);
        this->ports()->addPort(port_cmd_head_out_);
    }

    void updateHook() {
        velma_core_cs_task_cs_msgs::StatusHead st_head;
        if (port_status_head_in_.read(st_head) == RTT::NewData) {

            // Simple motion generation
            double abs_error = fabs(st_head.q_desired[0] - goal_pos_);
            if (abs_error < 0.01) {
                goal_pos_ = -goal_pos_;
                abs_error = fabs(st_head.q_desired[0] - goal_pos_);
                std::cout << "Set goal point to " << goal_pos_ << std::endl;
            }

            double vel = 0.5;   // rad/s
            double time_left = abs_error / vel;

            //st_head.status

            velma_core_cs_task_cs_msgs::CommandHead cmd;

            // time
            cmd.start = rtt_rosclock::host_now();

            // VelmaHeadTrajectoryPoint[50]
            // cmd.trj

            //std::cout << cmd.start.toSec() << ": current q_des: " << st_head.q_desired[0] << ", goal: " << goal_pos_
            //            << ", current dq_des: " << st_head.dq_desired[0] << ", time_left: "
            //            << time_left << std::endl;

            // Set the curernt point
            velma_core_cs_task_cs_msgs::VelmaHeadTrajectoryPoint& pt_current = cmd.trj[0];

            // float64[2]
            pt_current.positions[0] = st_head.q_desired[0];
            pt_current.positions[1] = st_head.q_desired[1];

            //float64[2]
            pt_current.velocities[0] = st_head.dq_desired[0];
            pt_current.velocities[1] = st_head.dq_desired[1];

            //float64[2]
            pt_current.accelerations[0] = 0;   // not used
            pt_current.accelerations[1] = 0;   // not used

            //duration
            pt_current.time_from_start = ros::Duration( 0 );

            //bool
            pt_current.use_velocities = true;

            //bool
            pt_current.use_accelerations = false;

            // Set the next point
            velma_core_cs_task_cs_msgs::VelmaHeadTrajectoryPoint& pt_next = cmd.trj[1];

            // float64[2]
            pt_next.positions[0] = goal_pos_;
            pt_next.positions[1] = st_head.q_desired[1];

            //float64[2]
            pt_next.velocities[0] = 0.0;    // stop at goal point
            pt_next.velocities[1] = 0.0;    // stop at goal point

            //float64[2]
            pt_next.accelerations[0] = 0;   // not used in this case
            pt_next.accelerations[1] = 0;   // not used in this case

            //duration
            pt_next.time_from_start = ros::Duration( time_left );

            //bool
            pt_next.use_velocities = true;

            //bool
            pt_next.use_accelerations = false;

            // uint32 
            cmd.count_trj = 2;  // there are two points in the trajectory: the current and the next

            // float64[2]
            cmd.path_tolerance[0] = 0;  // not used in this case
            cmd.path_tolerance[1] = 0;  // not used in this case

            // float64[2]
            cmd.goal_tolerance[0] = 0;  // not used in this case
            cmd.goal_tolerance[1] = 0;  // not used in this case

            // duration
            cmd.goal_time_tolerance = ros::Duration( 0 );

            // float64[2]
            cmd.stiffness[0] = 0;   // not used for head
            cmd.stiffness[1] = 0;   // not used for head

            port_cmd_head_out_.write(cmd);
        }
    }

private:
    // OROCOS ports
    RTT::InputPort<velma_core_cs_task_cs_msgs::StatusHead > port_status_head_in_;
    RTT::OutputPort<velma_core_cs_task_cs_msgs::CommandHead > port_cmd_head_out_;

    double goal_pos_;   // For simple motion generation
};

}   // namespace velma_core_cs_types

ORO_LIST_COMPONENT_TYPE(visual_servo_head_types::VisualServoComponent)

