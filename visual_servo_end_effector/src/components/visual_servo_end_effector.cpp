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
#include "velma_core_cs_task_cs_msgs/CommandCartImp.h"

#include "geometry_msgs/PoseArray.h"

namespace visual_servo_end_effector_types {

class VisualServoComponent: public RTT::TaskContext {
public:
    explicit VisualServoComponent(const std::string &name)
        : RTT::TaskContext(name)
        , port_status_r_wrist_in_("st_rWristPose_INPORT")
        , port_status_l_wrist_in_("st_lWristPose_INPORT")
        , port_q_all_in_("q_all_INPORT")
        , port_t_rArm_in_("t_rArm_INPORT")
        , port_t_lArm_in_("t_lArm_INPORT")
        , port_t_torso_in_("t_torso_INPORT")
        , port_cmd_cart_r_pose_out_("cmd_cart_r_pose_OUTPORT")
        , port_cmd_cart_l_pose_out_("cmd_cart_l_pose_OUTPORT")
        , port_pose_array_out_("pose_array_OUTPORT")
        , phase_(0.0)    // For simple motion generation
    {
        this->ports()->addPort(port_status_r_wrist_in_);
        this->ports()->addPort(port_status_l_wrist_in_);
        this->ports()->addPort(port_q_all_in_);
        this->ports()->addPort(port_t_rArm_in_);
        this->ports()->addPort(port_t_lArm_in_);
        this->ports()->addPort(port_t_torso_in_);
        this->ports()->addPort(port_cmd_cart_r_pose_out_);
        this->ports()->addPort(port_cmd_cart_l_pose_out_);
        this->ports()->addPort(port_pose_array_out_);

        pose_array_.poses.resize(10);
    }

    void updateHook() {
        geometry_msgs::Pose status_r_wrist_;
        if (port_status_r_wrist_in_.read(status_r_wrist_) == RTT::NewData) {
            AllJoints q_all;
            port_q_all_in_.read(q_all);
            std::cout << "POSITION: torso: " << q_all[0] << ", right arm: " << q_all[1] << ", "
                << q_all[2] << ", " << q_all[3] << ", " << q_all[4] << ", " << q_all[5]
                << ", " << q_all[6] << ", " << q_all[7] << ", left arm: " << q_all[8] << ", "
                << q_all[9] << ", " << q_all[10] << ", " << q_all[11] << ", " << q_all[12]
                << ", " << q_all[13] << ", " << q_all[14] << std::endl;

            ArmJoints t_rArm;
            port_t_rArm_in_.read(t_rArm);

            ArmJoints t_lArm;
            port_t_lArm_in_.read(t_lArm);

            double t_torso;
            port_t_torso_in_.read(t_torso);

            std::cout << "TORQUES: torso: " << t_torso << ", right arm: " << t_rArm[0] << ", "
                << t_rArm[1] << ", " << t_rArm[2] << ", " << t_rArm[3] << ", " << t_rArm[4]
                << ", " << t_rArm[5] << ", " << t_rArm[6] << ", left arm: " << t_lArm[0] << ", "
                << t_lArm[1] << ", " << t_lArm[2] << ", " << t_lArm[3] << ", " << t_lArm[4]
                << ", " << t_lArm[5] << ", " << t_lArm[6] << std::endl;

            // generate simple motion on a circle and limit the velocity
            const double subsystem_frequency = 20;      // [Hz]
            const double time_step = 1.0 / subsystem_frequency; // [s]
            const double radius = 0.05;                 // [m]

            const double max_vel = 0.1;                 // [m/s]
            const double rate = max_vel / radius;       // [rad/s]
            const double max_pos_change = max_vel * time_step;    // [m]

            const KDL::Vector center_pt(0.15, -0.4, 0.68);
            KDL::Vector goal_pt = center_pt +
                                        KDL::Vector(radius * cos(phase_), 0, radius * sin(phase_));

            phase_ += rate * time_step;

            KDL::Vector current_pt(status_r_wrist_.position.x, status_r_wrist_.position.y,
                                                                    status_r_wrist_.position.z);

            // limit the velocity
            KDL::Vector error = goal_pt - current_pt;
            if (error.Norm() > max_pos_change) {
                error.Normalize();
                goal_pt = current_pt + max_pos_change * error;
            }

            geometry_msgs::Pose wrist_r_goal_pose;
            wrist_r_goal_pose.position.x = goal_pt.x();
            wrist_r_goal_pose.position.y = goal_pt.y();
            wrist_r_goal_pose.position.z = goal_pt.z();

            wrist_r_goal_pose.orientation.x = -0.158736024039;
            wrist_r_goal_pose.orientation.y = 0.860902273027;
            wrist_r_goal_pose.orientation.z = -0.0341836807939;
            wrist_r_goal_pose.orientation.w = 0.482163485694;


            // rosmsg show velma_core_cs_task_cs_msgs/CommandCartImpTrjPose
            velma_core_cs_task_cs_msgs::CommandCartImpTrjPose cmd_r_pose;
            // time
            cmd_r_pose.start = rtt_rosclock::host_now();

            cartesian_trajectory_msgs::CartesianTrajectoryPoint& pt_current = cmd_r_pose.trj[0];
            pt_current.time_from_start = ros::Duration(0.0);
            pt_current.pose = status_r_wrist_;
            // pt_current.twist // this is ignored

            cartesian_trajectory_msgs::CartesianTrajectoryPoint& pt_next = cmd_r_pose.trj[1];
            pt_next.time_from_start = ros::Duration(time_step);
            pt_next.pose = wrist_r_goal_pose;

            cmd_r_pose.count = 2;

            // set path tolerance
            cmd_r_pose.path_tolerance.position.x = 0.1;
            cmd_r_pose.path_tolerance.position.y = 0.1;
            cmd_r_pose.path_tolerance.position.z = 0.1;

            cmd_r_pose.path_tolerance.rotation.x = 0.1;
            cmd_r_pose.path_tolerance.rotation.y = 0.1;
            cmd_r_pose.path_tolerance.rotation.z = 0.1;

            // set goal tolerance
            cmd_r_pose.goal_tolerance.position.x = 0.01;
            cmd_r_pose.goal_tolerance.position.y = 0.01;
            cmd_r_pose.goal_tolerance.position.z = 0.01;

            cmd_r_pose.goal_tolerance.rotation.x = 0.01;
            cmd_r_pose.goal_tolerance.rotation.y = 0.01;
            cmd_r_pose.goal_tolerance.rotation.z = 0.01;

            // duration - must be set to 0, to ignore old time stamp
            cmd_r_pose.goal_time_tolerance = ros::Duration( 0 );

            port_cmd_cart_r_pose_out_.write(cmd_r_pose);

            pose_array_.header.stamp = rtt_rosclock::host_now();
            pose_array_.header.frame_id = "torso_base";
            for (int i = 0; i < 10; ++i) {
                pose_array_.poses[i] = wrist_r_goal_pose;
                double offset = double(i) * 0.1;
                pose_array_.poses[i].position.x += offset;
            }
            port_pose_array_out_.write(pose_array_);
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

    RTT::OutputPort<velma_core_cs_task_cs_msgs::CommandCartImpTrjPose > port_cmd_cart_r_pose_out_;
    RTT::OutputPort<velma_core_cs_task_cs_msgs::CommandCartImpTrjPose > port_cmd_cart_l_pose_out_;

    // Ports used for visualization in ROS
    geometry_msgs::PoseArray pose_array_;
    RTT::OutputPort<geometry_msgs::PoseArray > port_pose_array_out_;

    double phase_;   // For simple motion generation
};

}   // namespace velma_core_cs_types

ORO_LIST_COMPONENT_TYPE(visual_servo_end_effector_types::VisualServoComponent)
