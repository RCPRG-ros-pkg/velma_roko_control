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

#include <Eigen/Dense>

#include <rtt/deployment/ComponentLoader.hpp>

#include <rtt_roscomm/rostopic.h>
#include <rtt_roscomm/rosservice.h>
#include <std_msgs/typekit/String.h>
#include <std_srvs/Empty.h>

#include <simulation_control_msgs/EnableSim.h>

namespace visual_servo_head_types {

class VisualServoComponent: public RTT::TaskContext {
public:
    explicit VisualServoComponent(const std::string &name)
        : RTT::TaskContext(name, PreOperational)
        , port_status_head_in_("st_head_INPORT")
        , port_q_in_("q_INPORT")
        , port_cmd_head_out_("cmd_head_OUTPORT")
    {
        this->ports()->addPort(port_status_head_in_);
        this->ports()->addPort(port_q_in_);
        this->ports()->addPort(port_cmd_head_out_);
    }

    virtual bool configureHook() {
        buf_size_ = 10000;
        buf_idx_ = 0;
        buf_a_.resize(buf_size_);
        buf_b_.resize(buf_size_);
        buf_c_.resize(buf_size_);

        return true;
    }

    virtual bool startHook() {
        is_first_step_ = true;
        return true;
    }

    virtual void stopHook() {
        std::cout << buf_size_ << " " << buf_idx_ << std::endl;
        for (int i = 0; i < buf_idx_; ++i) {
            std::cout << buf_a_[i] << " " << buf_b_[i] << " " << buf_c_[i] << std::endl;
        }
    }

    virtual void updateHook() {

        port_q_in_.read(q_in_);

        velma_core_cs_task_cs_msgs::StatusHead st_head;
        if (port_status_head_in_.read(st_head) == RTT::NewData) {

            // Simple motion generation
            if (is_first_step_) {
                is_first_step_ = false;
                goal_pos_ = st_head.q_desired[0];
                if (goal_pos_ < 0) {
                    // Go up
                    phase_ = 0;
                }
                else {
                    // Go down
                    phase_ = 1;
                }
            }

            // Save values for printing
            //std::cout << st_head.q_desired[0] << ", " << goal_pos_ << ", " << q_in_(15) << std::endl;
            if (buf_idx_ < buf_size_) {
                buf_a_[buf_idx_] = st_head.q_desired[0];
                buf_b_[buf_idx_] = goal_pos_;
                buf_c_[buf_idx_] = q_in_(15);
                ++buf_idx_;
            }

            double goal_vel;
            double vel = 0.5;   // rad/s
            double frequency = 500.0;

            double goal_pos2, goal_pos3;
            if (phase_ == 0) {
                goal_pos_ += vel / frequency;
                goal_pos2 = goal_pos_ + vel / frequency;
                goal_pos3 = goal_pos2 + vel / frequency;
                goal_vel = vel;
            }
            else {
                goal_pos_ -= vel / frequency;
                goal_pos2 = goal_pos_ - vel / frequency;
                goal_pos3 = goal_pos2 - vel / frequency;
                goal_vel = -vel;
            }

            if (goal_pos_ > 1.0) {
                // Go down
                phase_ = 1;
                std::cout << "Set goal point to -1" << std::endl;
            }
            else if (goal_pos_ < -1.0) {
                // Go up
                phase_ = 0;
                std::cout << "Set goal point to 1" << std::endl;
            }

            velma_core_cs_task_cs_msgs::CommandHead cmd;

            // Use iteration index
            cmd.start = ros::Time(0);

            // Set the next point
            velma_core_cs_task_cs_msgs::VelmaHeadTrajectoryPoint& pt_next = cmd.trj[0];
            velma_core_cs_task_cs_msgs::VelmaHeadTrajectoryPoint& pt_next2 = cmd.trj[1];
            velma_core_cs_task_cs_msgs::VelmaHeadTrajectoryPoint& pt_next3 = cmd.trj[2];

            // float64[2]
            pt_next.positions[0] = goal_pos_;
            pt_next.positions[1] = st_head.q_desired[1];

            //float64[2]
            pt_next.velocities[0] = goal_vel;    // stop at goal point
            pt_next.velocities[1] = 0.0;    // stop at goal point

            //float64[2]
            pt_next.accelerations[0] = 0;   // not used in this case
            pt_next.accelerations[1] = 0;   // not used in this case

            //duration
            //pt_next.time_from_start = ros::Duration( time_left );

            // Use iteration index: this cycle
            pt_next.time_from_start = ros::Duration( 0, 0 );

            //bool
            pt_next.use_velocities = true;

            //bool
            pt_next.use_accelerations = false;

            // Two points for the next two iterations - they are never executed, bacause
            // we send a new command in every step
            pt_next2.positions[0] = goal_pos2;
            pt_next2.positions[1] = st_head.q_desired[1];
            pt_next2.velocities[0] = goal_vel;    // stop at goal point
            pt_next2.velocities[1] = 0.0;    // stop at goal point
            pt_next2.accelerations[0] = 0;   // not used in this case
            pt_next2.accelerations[1] = 0;   // not used in this case
            pt_next2.time_from_start = ros::Duration( 1, 0 );
            pt_next2.use_velocities = true;
            pt_next2.use_accelerations = false;


            pt_next3.positions[0] = goal_pos3;
            pt_next3.positions[1] = st_head.q_desired[1];
            pt_next3.velocities[0] = goal_vel;    // stop at goal point
            pt_next3.velocities[1] = 0.0;    // stop at goal point
            pt_next3.accelerations[0] = 0;   // not used in this case
            pt_next3.accelerations[1] = 0;   // not used in this case
            pt_next3.time_from_start = ros::Duration( 2, 0 );
            pt_next3.use_velocities = true;
            pt_next3.use_accelerations = false;


            // uint32 
            cmd.count_trj = 3;  // there are two points in the trajectory: the current and the next

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
        else {
            std::cout << "no new data" << std::endl;
        }
    }

private:
    // OROCOS ports
    RTT::InputPort<velma_core_cs_task_cs_msgs::StatusHead > port_status_head_in_;

    typedef Eigen::Matrix<double, 33, 1>  VectorQ;
    VectorQ q_in_;
    RTT::InputPort<VectorQ > port_q_in_;

    RTT::OutputPort<velma_core_cs_task_cs_msgs::CommandHead > port_cmd_head_out_;

    double goal_pos_;   // For simple motion generation
    int phase_;
    bool is_first_step_;

    // Used for printing only
    int buf_size_;
    int buf_idx_;
    std::vector<double> buf_a_;
    std::vector<double> buf_b_;
    std::vector<double> buf_c_;
};

}   // namespace velma_core_cs_types

ORO_LIST_COMPONENT_TYPE(visual_servo_head_types::VisualServoComponent)
