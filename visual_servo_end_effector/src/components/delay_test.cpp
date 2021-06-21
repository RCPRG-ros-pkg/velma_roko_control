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

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include "geometry_msgs/Pose.h"
#include "velma_core_cs_task_cs_msgs/CommandJntImp.h"

#include "geometry_msgs/PoseArray.h"

namespace visual_servo_end_effector_types {

class DelayTestComponent: public RTT::TaskContext {
public:
    explicit DelayTestComponent(const std::string &name)
        : RTT::TaskContext(name)
        , port_status_r_wrist_in_("st_rWristPose_INPORT")
        , port_status_l_wrist_in_("st_lWristPose_INPORT")
        , port_q_all_in_("q_all_INPORT")
        , port_t_rArm_in_("t_rArm_INPORT")
        , port_t_lArm_in_("t_lArm_INPORT")
        , port_t_torso_in_("t_torso_INPORT")
        , port_cmd_jnt_out_("cmd_jnt_OUTPORT")
        , port_pose_array_out_("pose_array_OUTPORT")
        , state_(0)
    {
        this->ports()->addPort(port_status_r_wrist_in_);
        this->ports()->addPort(port_status_l_wrist_in_);
        this->ports()->addPort(port_q_all_in_);
        this->ports()->addPort(port_t_rArm_in_);
        this->ports()->addPort(port_t_lArm_in_);
        this->ports()->addPort(port_t_torso_in_);
        this->ports()->addPort(port_cmd_jnt_out_);
        this->ports()->addPort(port_pose_array_out_);

        pose_array_.poses.resize(10);

        finish_time_ = rtt_rosclock::host_now();
    }

    void updateHook() {
        geometry_msgs::Pose status_r_wrist;
        if (port_status_r_wrist_in_.read(status_r_wrist) != RTT::NewData) {
            return;
        }

        usleep(1000*(25*5)); // <-- odkomentowanie TEGO powoduje ignorowanie wiadomosci

        if (rtt_rosclock::host_now() > finish_time_) {

            ros::Time time_a = rtt_rosclock::host_now();

            velma_core_cs_task_cs_msgs::CommandJntImp cmd_jnt;

            cmd_jnt.start = rtt_rosclock::host_now();

            std::cout << "time diff: " << (cmd_jnt.start - time_a).toSec() << std::endl;
            cmd_jnt.count_trj = 1;
            cmd_jnt.trj[0].time_from_start = ros::Duration( 6.0 );

            finish_time_ = cmd_jnt.start + cmd_jnt.trj[0].time_from_start;

            switch (state_) {
                case 0:
                    cmd_jnt.trj[0].positions={ 0.5, -0.3, -1.5, 1.25, 0.85, 0, -0.75, 0, 0.3, 1.5, -1.25, -0.85, 0, 0.75, 0};
                    state_ = 1;
                    break;
                default:
                    // Initial position
                    cmd_jnt.trj[0].positions={ 0, -0.3, -1.8, 1.25, 0.85, 0, -0.5, 0, 0.3, 1.8, -1.25, -0.85, 0, 0.5, 0};
                    state_ = 0;
                    break;
            }

            cmd_jnt.path_tolerance = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
            cmd_jnt.goal_tolerance = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
            cmd_jnt.goal_time_tolerance = ros::Duration( 0 );
            cmd_jnt.stiffness = {1000, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600};

            port_cmd_jnt_out_.write(cmd_jnt);

            // Send the last point in the trajectory
            //port_xce_end_out_.write(active_trajectory_.trj[active_trajectory_.count-1].pose);

            //pose_array_.header.stamp = rtt_rosclock::host_now();
            //pose_array_.header.frame_id = "torso_base";
            //for (int i = 0; i < 10; ++i) {
            //    pose_array_.poses[i] = wrist_r_goal_pose;
            //    double offset = double(i) * 0.1;
            //    pose_array_.poses[i].position.x += offset;
            //}
            //port_pose_array_out_.write(pose_array_);
        }
    }

private:
    // OROCOS ports
    RTT::InputPort<geometry_msgs::Pose > port_status_r_wrist_in_;
    RTT::InputPort<geometry_msgs::Pose > port_status_l_wrist_in_;

    typedef Eigen::Matrix<double, 33, 1> AllJoints;
    RTT::InputPort<AllJoints > port_q_all_in_;

    typedef Eigen::Matrix<double, 7, 1> ArmJoints;
    RTT::InputPort<ArmJoints > port_t_rArm_in_;
    RTT::InputPort<ArmJoints > port_t_lArm_in_;
    RTT::InputPort<double > port_t_torso_in_;

    // Fake Cartesian trajectory command (from the task agent group)
    geometry_msgs::Pose starting_r_wrist_pose_;

    // Data for manipulation strategy component: the pose at the catch moment
    //RTT::OutputPort<geometry_msgs::Pose > port_xce_end_out_;


    RTT::OutputPort<velma_core_cs_task_cs_msgs::CommandJntImp > port_cmd_jnt_out_;

    // Ports used for visualization in ROS
    geometry_msgs::PoseArray pose_array_;
    RTT::OutputPort<geometry_msgs::PoseArray > port_pose_array_out_;

    int state_;
    ros::Time finish_time_;
};

}   // namespace velma_core_cs_types

ORO_LIST_COMPONENT_TYPE(visual_servo_end_effector_types::DelayTestComponent)
