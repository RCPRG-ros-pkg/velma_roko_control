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

class VisualServoFakeComponent: public RTT::TaskContext {
public:
    explicit VisualServoFakeComponent(const std::string &name)
        : RTT::TaskContext(name)
        , port_status_r_wrist_in_("st_rWristPose_INPORT")
        , port_status_l_wrist_in_("st_lWristPose_INPORT")
        , port_q_all_in_("q_all_INPORT")
        , port_t_rArm_in_("t_rArm_INPORT")
        , port_t_lArm_in_("t_lArm_INPORT")
        , port_t_torso_in_("t_torso_INPORT")
        , port_cmd_cart_r_pose_out_("cmd_cart_r_pose_OUTPORT")
        , port_cmd_cart_l_pose_out_("cmd_cart_l_pose_OUTPORT")
        , port_cmd_cart_r_pose_in_("cmd_cart_r_pose_INPORT")
        , port_xce_end_out_("xce_end_OUTPORT")
        , port_pose_array_out_("pose_array_OUTPORT")
        , command_active_(false)
    {
        this->ports()->addPort(port_status_r_wrist_in_);
        this->ports()->addPort(port_status_l_wrist_in_);
        this->ports()->addPort(port_q_all_in_);
        this->ports()->addPort(port_t_rArm_in_);
        this->ports()->addPort(port_t_lArm_in_);
        this->ports()->addPort(port_t_torso_in_);
        this->ports()->addPort(port_cmd_cart_r_pose_out_);
        this->ports()->addPort(port_cmd_cart_l_pose_out_);
        this->ports()->addPort(port_cmd_cart_r_pose_in_);
        this->ports()->addPort(port_xce_end_out_);
        this->ports()->addPort(port_pose_array_out_);

        pose_array_.poses.resize(10);
    }

    void updateHook() {
        geometry_msgs::Pose status_r_wrist;
        if (port_status_r_wrist_in_.read(status_r_wrist) != RTT::NewData) {
            return;
        }

        if (!command_active_) {
            // There is no active command, so we can listen for new commands
            if (port_cmd_cart_r_pose_in_.read(active_trajectory_) == RTT::NewData) {
                // Save the commanded trajectory for further 20Hz intepolation
                starting_r_wrist_pose_ = status_r_wrist;
                // Update the start time
                active_trajectory_.start = rtt_rosclock::host_now();
                // Set a flag for active command
                command_active_ = true;
                std::cout << "Received a new command" << std::endl;
            }
        }

        if (command_active_) {
            // There is an active command, so interpolate for next 10 cycles 20Hz
            const int cycles_forward = 10;
            const ros::Duration cycle_interval(0.05);
            ros::Time current_time = rtt_rosclock::host_now();

            velma_core_cs_task_cs_msgs::CommandCartImpTrjPose trj_out;
            trj_out.start = current_time;

            bool finish = false;
            ros::Duration time_from_start(0.0);
            for (int i = 0; i < cycles_forward; ++i) {
                time_from_start += cycle_interval;
                if (!getInterpolatedPointAtTime(current_time + time_from_start, trj_out.trj[i].pose)
                        && i == 0) {
                    finish = true;
                }
                trj_out.trj[i].time_from_start = time_from_start;
            }
            
            if (finish) {
                // There is nothing more to interpolate
                command_active_ = false;
                std::cout << "Finished motion" << std::endl;
                return;
            }


            trj_out.count = cycles_forward;

            // set path tolerance
            trj_out.path_tolerance.position.x = 0.1;
            trj_out.path_tolerance.position.y = 0.1;
            trj_out.path_tolerance.position.z = 0.1;

            trj_out.path_tolerance.rotation.x = 0.1;
            trj_out.path_tolerance.rotation.y = 0.1;
            trj_out.path_tolerance.rotation.z = 0.1;

            // set goal tolerance
            trj_out.goal_tolerance.position.x = 0.01;
            trj_out.goal_tolerance.position.y = 0.01;
            trj_out.goal_tolerance.position.z = 0.01;

            trj_out.goal_tolerance.rotation.x = 0.01;
            trj_out.goal_tolerance.rotation.y = 0.01;
            trj_out.goal_tolerance.rotation.z = 0.01;

            // duration - must be set to 0, to ignore old time stamp
            trj_out.goal_time_tolerance = ros::Duration( 0 );

            port_cmd_cart_r_pose_out_.write(trj_out);

            // Send the last point in the trajectory
            port_xce_end_out_.write(active_trajectory_.trj[active_trajectory_.count-1].pose);

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

    bool getInterpolatedPointAtTime(const ros::Time& time, geometry_msgs::Pose& out_pose) const {

        // Get the current segment
        // trj pt idx:     0    1    2    3    4    5
        // trj times:      2    4    6    8    10   12
        // current time: 1    3    5    7
        // next pt idx:  0    1    2    3
        int next_pt_idx = -1;   // Set to -1 to detect the 'finish' condition
        for (int pt_idx = 0; pt_idx < active_trajectory_.count; ++pt_idx) {
            ros::Time pt_time = active_trajectory_.start
                                            + active_trajectory_.trj[pt_idx].time_from_start;
            if (time < pt_time) {
                next_pt_idx = pt_idx;
            }
            else {
                break;
            }
        }

        cartesian_trajectory_msgs::CartesianTrajectoryPoint p0;
        if (next_pt_idx < 0) {
            // All points in the trajectory are old
            // Set the pose to the last point in the trajectory
            out_pose = active_trajectory_.trj[active_trajectory_.count-1].pose;
            return false;
        }
        else if (next_pt_idx == 0) {
            // Interpolate from the starting pose to the first point
            p0.pose = starting_r_wrist_pose_;
            p0.time_from_start = ros::Duration(0.0);
        }
        else { // next_pt_idx > 0
            // Interpolate between two points
            p0 = active_trajectory_.trj[next_pt_idx-1];
        }

        const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p1 =
                                                        active_trajectory_.trj[next_pt_idx];
        out_pose = interpolate(p0, p1, time);
        return true;
    }


    geometry_msgs::Pose interpolate(const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p0,
                                    const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p1,
                                                                            ros::Time t) const {
        geometry_msgs::Pose pose;

        ros::Time t0 = active_trajectory_.start + p0.time_from_start;
        ros::Time t1 = active_trajectory_.start + p1.time_from_start;

        pose.position.x = interpolate(p0.pose.position.x, p1.pose.position.x,
        t0.toSec(), t1.toSec(), t.toSec());
        pose.position.y = interpolate(p0.pose.position.y, p1.pose.position.y,
        t0.toSec(), t1.toSec(), t.toSec());
        pose.position.z = interpolate(p0.pose.position.z, p1.pose.position.z,
        t0.toSec(), t1.toSec(), t.toSec());

        Eigen::Quaterniond q0(p0.pose.orientation.w, p0.pose.orientation.x,
        p0.pose.orientation.y, p0.pose.orientation.z);
        Eigen::Quaterniond q1(p1.pose.orientation.w, p1.pose.orientation.x,
        p1.pose.orientation.y, p1.pose.orientation.z);

        double a = interpolate(0.0, 1.0, t0.toSec(), t1.toSec(), t.toSec());
        Eigen::Quaterniond q = q0.slerp(a, q1);
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();

        return pose;
    }

    double interpolate(double p0, double p1, double t0, double t1, double t) const {
        return (p0 + (p1 - p0) * (t - t0) / (t1 - t0));
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
    velma_core_cs_task_cs_msgs::CommandCartImpTrjPose active_trajectory_;
    RTT::InputPort<velma_core_cs_task_cs_msgs::CommandCartImpTrjPose > port_cmd_cart_r_pose_in_;
    bool command_active_;

    // Data for manipulation strategy component: the pose at the catch moment
    RTT::OutputPort<geometry_msgs::Pose > port_xce_end_out_;


    RTT::OutputPort<velma_core_cs_task_cs_msgs::CommandCartImpTrjPose > port_cmd_cart_r_pose_out_;
    RTT::OutputPort<velma_core_cs_task_cs_msgs::CommandCartImpTrjPose > port_cmd_cart_l_pose_out_;

    // Ports used for visualization in ROS
    geometry_msgs::PoseArray pose_array_;
    RTT::OutputPort<geometry_msgs::PoseArray > port_pose_array_out_;
};

}   // namespace velma_core_cs_types

ORO_LIST_COMPONENT_TYPE(visual_servo_end_effector_types::VisualServoFakeComponent)
