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
#include "std_msgs/Int32.h"

#include "geometry_msgs/PoseArray.h"


static Eigen::Quaterniond msgToEigen(const geometry_msgs::Quaternion& q) {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

static Eigen::Vector3d msgToEigen(const geometry_msgs::Point& p) {
    return Eigen::Vector3d(p.x, p.y, p.z);
}

static void setImpedanceMsg(double kx, double ky, double kz, double krx, double kry, double krz,
                            double dx, double dy, double dz, double drx, double dry, double drz,
                            cartesian_trajectory_msgs::CartesianImpedance& out_msg) {
    out_msg.stiffness.force.x = kx;
    out_msg.stiffness.force.y = ky;
    out_msg.stiffness.force.z = kz;
    out_msg.stiffness.torque.x = krx;
    out_msg.stiffness.torque.y = kry;
    out_msg.stiffness.torque.z = krz;

    out_msg.damping.force.x = dx;
    out_msg.damping.force.y = dy;
    out_msg.damping.force.z = dz;
    out_msg.damping.torque.x = drx;
    out_msg.damping.torque.y = dry;
    out_msg.damping.torque.z = drz;
}

namespace visual_servo_end_effector_types {

class ManipulationStrategyComponent: public RTT::TaskContext {
public:
    explicit ManipulationStrategyComponent(const std::string &name)
        : RTT::TaskContext(name)
        , port_status_r_wrist_in_("st_rWristPose_INPORT")
        , port_status_l_wrist_in_("st_lWristPose_INPORT")
        , port_q_all_in_("q_all_INPORT")
        , port_cmd_cart_r_imp_out_("cmd_cart_r_imp_OUTPORT")
        , port_cmd_cart_l_imp_out_("cmd_cart_l_imp_OUTPORT")
        , port_xce_end_in_("xce_end_INPORT")
        , port_pose_array_out_("pose_array_OUTPORT")
        , port_strategy_mode_out_("strategy_mode_OUTPORT")
        , phase_(0.0)    // For simple motion generation
        , strategy_mode_(STRATEGY_FAR)
    {
        this->ports()->addPort(port_status_r_wrist_in_);
        this->ports()->addPort(port_status_l_wrist_in_);
        this->ports()->addPort(port_q_all_in_);
        this->ports()->addPort(port_xce_end_in_);
        this->ports()->addPort(port_cmd_cart_r_imp_out_);
        this->ports()->addPort(port_cmd_cart_l_imp_out_);
        this->ports()->addPort(port_pose_array_out_);
        this->ports()->addPort(port_strategy_mode_out_);

        pose_array_.poses.resize(10);
    }

    void updateHook() {
        geometry_msgs::Pose status_r_wrist;
        if (port_status_r_wrist_in_.read(status_r_wrist) != RTT::NewData) {
            return;
        }

        geometry_msgs::Pose xce_end_in;
        if (port_xce_end_in_.read(xce_end_in) == RTT::NewData) {

            // Compare the current pose to the catch pose
            Eigen::Quaterniond q0 = msgToEigen(xce_end_in.orientation);

            Eigen::Quaterniond q1 = msgToEigen(status_r_wrist.orientation);

            double ang_dist = fabs( q0.angularDistance(q1) );

            Eigen::Vector3d p0 = msgToEigen(xce_end_in.position);
            Eigen::Vector3d p1 = msgToEigen(status_r_wrist.position);

            double lin_dist = (p0-p1).norm();

            const double ang_tol_low = 15.0/180.0*3.1415;
            const double lin_tol_low = 0.05;
            const double ang_tol_high = 25.0/180.0*3.1415;
            const double lin_tol_high = 0.1;
            if (strategy_mode_ == STRATEGY_NEAR) {
                if (ang_dist > ang_tol_high || lin_dist > lin_tol_high) {
                    strategy_mode_ = STRATEGY_FAR;
                    velma_core_cs_task_cs_msgs::CommandCartImpTrjImp imp_out;
                    imp_out.start = rtt_rosclock::host_now();
                    imp_out.trj[0].time_from_start = ros::Duration(0.5);
                    setImpedanceMsg(1000, 1000, 1000, 300, 300, 300,
                                    0.7, 0.7, 0.7, 0.7, 0.7, 0.7, imp_out.trj[0].impedance);
                    imp_out.count = 1;
                    port_cmd_cart_r_imp_out_.write(imp_out);
                    std::cout << "Commanded high stiffness" << std::endl;
                }
            }
            else if (strategy_mode_ == STRATEGY_FAR) {
                if (ang_dist < ang_tol_low && lin_dist < lin_tol_low) {
                    strategy_mode_ = STRATEGY_NEAR;
                    velma_core_cs_task_cs_msgs::CommandCartImpTrjImp imp_out;
                    imp_out.start = rtt_rosclock::host_now();
                    imp_out.trj[0].time_from_start = ros::Duration(0.5);
                    //setImpedanceMsg(50, 50, 50, 300, 300, 300,
                    //                0.7, 0.7, 0.7, 0.7, 0.7, 0.7, imp_out.trj[0].impedance);
                    setImpedanceMsg(50, 50, 50, 15, 15, 15,
                                    0.7, 0.7, 0.7, 0.7, 0.7, 0.7, imp_out.trj[0].impedance);
                    imp_out.count = 1;
                    port_cmd_cart_r_imp_out_.write(imp_out);
                    std::cout << "Commanded low stiffness" << std::endl;
                }
            }
            else {
                std::cout << "Wrong strategy id: " << strategy_mode_ << std::endl;
            }

            std_msgs::Int32 strategy_mode_out;
            strategy_mode_out.data = strategy_mode_;
            port_strategy_mode_out_.write( strategy_mode_out );
        }
    }

private:
    // OROCOS ports
    RTT::InputPort<geometry_msgs::Pose > port_status_r_wrist_in_;
    RTT::InputPort<geometry_msgs::Pose > port_status_l_wrist_in_;

    typedef Eigen::Matrix<double, 33, 1> AllJoints;
    RTT::InputPort<AllJoints > port_q_all_in_;

    // Data for manipulation strategy component: the pose at the catch moment
    RTT::InputPort<geometry_msgs::Pose > port_xce_end_in_;

    RTT::OutputPort<velma_core_cs_task_cs_msgs::CommandCartImpTrjImp > port_cmd_cart_r_imp_out_;
    RTT::OutputPort<velma_core_cs_task_cs_msgs::CommandCartImpTrjImp > port_cmd_cart_l_imp_out_;

    // Ports used for visualization in ROS
    geometry_msgs::PoseArray pose_array_;
    RTT::OutputPort<geometry_msgs::PoseArray > port_pose_array_out_;

    double phase_;   // For simple motion generation
    enum {STRATEGY_NEAR=0, STRATEGY_FAR=1};
    int strategy_mode_;

    RTT::OutputPort<std_msgs::Int32 > port_strategy_mode_out_;
};

}   // namespace velma_core_cs_types

ORO_LIST_COMPONENT_TYPE(visual_servo_end_effector_types::ManipulationStrategyComponent)
