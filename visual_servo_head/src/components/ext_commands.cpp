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

#include "std_msgs/Int32.h"

namespace visual_servo_head_types {

class ExtCommandsComponent: public RTT::TaskContext {
public:
    explicit ExtCommandsComponent(const std::string &name)
        : RTT::TaskContext(name)
        , port_cmd_in_("cmd_INPORT")
        , recvVisualServoCmd_(false)
        , recvCartImpObstCmd_(false)
        , recvIdleCmd_(false)
    {
        this->ports()->addPort(port_cmd_in_);
        this->addAttribute("recvVisualServoCmd", recvVisualServoCmd_);
        this->addAttribute("recvCartImpObstCmd", recvCartImpObstCmd_);
        this->addAttribute("recvIdleCmd", recvIdleCmd_);
    }

    void updateHook() {
        std_msgs::Int32 cmd;
        if (port_cmd_in_.read(cmd) == RTT::NewData) {
            if (cmd.data == 0) {
                std::cout << "ExtCommandsComponent: Received command: idle" << std::endl;
                recvVisualServoCmd_ = false;
                recvCartImpObstCmd_ = false;
                recvIdleCmd_ = true;
            }
            else if (cmd.data == 1) {
                std::cout << "ExtCommandsComponent: Received command: visual_servo" << std::endl;
                recvVisualServoCmd_ = true;
                recvCartImpObstCmd_ = false;
                recvIdleCmd_ = false;
            }
            else if (cmd.data == 2) {
                std::cout << "ExtCommandsComponent: Received command: cart_imp_obst" << std::endl;
                recvVisualServoCmd_ = false;
                recvCartImpObstCmd_ = true;
                recvIdleCmd_ = false;
            }
            else {
                std::cout << "ExtCommandsComponent: Received unknown command" << std::endl;
            }
        }
    }

private:
    // OROCOS ports
    RTT::InputPort<std_msgs::Int32 > port_cmd_in_;
    bool recvVisualServoCmd_;
    bool recvCartImpObstCmd_;
    bool recvIdleCmd_;
};

}   // namespace velma_core_cs_types

ORO_LIST_COMPONENT_TYPE(visual_servo_head_types::ExtCommandsComponent)
