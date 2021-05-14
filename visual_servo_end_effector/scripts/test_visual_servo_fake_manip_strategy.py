#!/usr/bin/env python

## This node simulates perception of simulated Velma robot.

# Copyright (c) 2020, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import rospkg
import os
import copy
import PyKDL
import math
import numpy as np
import threading

import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import tf_conversions.posemath as pm
from std_msgs.msg import Int32, Header

from velma_core_cs_task_cs_msgs.msg import CommandCartImpTrjPose

from velma_common import VelmaInterface, symmetricalConfiguration
from rcprg_ros_utils import exitError

from velma_core_cs_ve_body_msgs.msg import Status

'''
roslaunch roko_nodes velma_system_roko.launch world_name:=worlds/grasping_stiff.world
roslaunch visual_servo_end_effector visual_servo_end_effector.launch
'''

def printFrame(T):
    q = T.M.GetQuaternion()
    print('PyKDL.Frame(PyKDL.Rotation.Quaternion({}, {}, {}, {}), PyKDL.Vector({}, {}, {}))'.format(
            q[0], q[1], q[2], q[3], T.p.x(), T.p.y(), T.p.z()))

class VisualServoTests:
    def __init__(self):
        rospy.init_node('test_roko_2_1', anonymous=False)
        rospy.sleep(1)
        self.__q_map_starting = symmetricalConfiguration( {'torso_0_joint':0,
            'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8, 'right_arm_2_joint':1.25,
            'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
            'right_arm_6_joint':0} )

        self.__q_map_2 = self.__q_map_starting.copy()
        self.__q_map_2['right_arm_0_joint'] = -0.8
        self.__q_map_2['right_arm_3_joint'] = 1.6

        self.__arm_side = 'right'

        self.__velma = VelmaInterface()

        print "Waiting for VelmaInterface initialization..."
        if not self.__velma.waitForInit(timeout_s=10.0):
            print "Could not initialize VelmaInterface\n"
            exitError(3)

        print "Initialization ok!\n"

        diag = self.__velma.getCoreCsDiag()
        if (not diag.inStateCartImp()) and (not diag.inStateJntImp()):
            exitError(4, msg='Velma should be in cart_imp or jnt_imp state')

        self.__vs_trj_pub = rospy.Publisher('/visual_servo_end_effector/trajectory_in',
                                                            CommandCartImpTrjPose, queue_size=10)

        self.__vs_cmd_pub = rospy.Publisher('/visual_servo_end_effector/cmd_in', Int32, queue_size=10)

        self.__info_pub = rospy.Publisher('/roko_test_2_1/info_out', Header, queue_size=10)

        self.__mutex = threading.Lock()
        self.clearData()
        self.__st_sub = rospy.Subscriber('/velma_core_cs/b_st', Status, self.__statusCallback)
        self.__strategy_sub = rospy.Subscriber('/visual_servo_end_effector/strategy_mode_out', Int32, self.__strategyCallback)

    def __statusCallback(self, data):
        with self.__mutex:
            self.__data.append( ('ft', rospy.Time.now(), data.rFt.sfw) )

    def __strategyCallback(self, data):
        with self.__mutex:
            self.__data.append( ('strategy', rospy.Time.now(), data.data) )

    def publishInfo(self, text):
        hdr = Header()
        hdr.stamp = rospy.Time.now()
        hdr.frame_id = text
        self.__info_pub.publish( hdr )

        with self.__mutex:
            self.__data.append( ('info', hdr.stamp, text) )

    def clearData(self):
        with self.__mutex:
            self.__data = []

    def writeData(self, filename):
        with self.__mutex:
            data = copy.copy(self.__data)
        data = sorted(data, key=lambda x: x[1].to_sec())
        with open(filename, 'w') as f:
            for d in data:
                if d[0] == 'ft':
                    f.write('ft;{};{};{};{};{};{};{}\n'.format(d[1].to_sec(), d[2].force.x, d[2].force.y,
                                    d[2].force.z, d[2].torque.x, d[2].torque.y, d[2].torque.z))
                elif d[0] == 'strategy':
                    f.write('strategy;{};{}\n'.format(d[1].to_sec(), d[2]))
                elif d[0] == 'info':
                    f.write('info;{};{}\n'.format(d[1].to_sec(), d[2]))

    def makeTest(self, vs_mode, y_offset):
        assert vs_mode in ('visual_servo', 'visual_servo_manip_strategy')

        self.publishInfo( 'begin test: vs_mode: {}, y_offset: {}'.format(vs_mode, y_offset) )

        print("Opening fingers...")
        hand_q = [math.radians(0), math.radians(0), math.radians(0), 0]
        self.__velma.moveHand(self.__arm_side, hand_q, [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=False)
        if self.__velma.waitForHand(self.__arm_side) != 0:
            exitError(6)

        max_vel = math.radians(15.0)

        #print("Moving to the starting position...")
        #self.__velma.moveJoint(self.__q_map_starting, None, max_vel=max_vel, start_time=0.5, position_tol=15.0/180.0*math.pi)
        #error = self.__velma.waitForJoint()
        #if error != 0:
        #    exitError(6, msg='The action moveJoint ended with error code: {}'.format(error))

        #move_arm = False
        #move_arm_back = False
        move_arm = True
        move_arm_back = True

        force_limit = [2000, 2000, 2000, 2000]
        #hand_dest_q = [math.radians(90), math.radians(90), math.radians(90), 0]
        hand_dest_q = [math.radians(60), math.radians(60), math.radians(60), 0]
        #hand_dest_q = [math.radians(50), math.radians(50), math.radians(50), 0]

        if move_arm:
            print("Moving to the intermadiate position...")
            self.__velma.moveJoint(self.__q_map_2, None, max_vel=max_vel, start_time=0.5, position_tol=15.0/180.0*math.pi)
            error = self.__velma.waitForJoint()
            if error != 0:
                exitError(6, msg='The action moveJoint ended with error code: {}'.format(error))

            dT_B_Wr = PyKDL.Frame(
                            PyKDL.Rotation.Quaternion(0.0060530740706, 0.479021471557,
                            -0.0212848492887, 0.877524213497),
                            PyKDL.Vector(0.320550010456, -0.352669024908+y_offset, 1.03579754614))

            max_vel_lin = 0.2
            max_vel_rot = 0.2
            T_B_Wr = self.__velma.getTf('B', 'Wr', time=None, timeout_s=1.0)

            dT_B_Wr_init = PyKDL.Frame(dT_B_Wr.M, T_B_Wr.p)
            #tx = T_B_Wr.p.x()
            #ty = T_B_Wr.p.y()
            #tz = T_B_Wr.p.z()
            #qx, qy, qz, qw = T_B_Wr.M.GetQuaternion()
            #print('PyKDL.Frame(PyKDL.Rotation.Quaternion({}, {}, {}, {}), PyKDL.Vector({}, {}, {}))'.format(
            #                                                            qx, qy, qz, qw, tx, ty, tz))
            #dT_B_Wr = T_B_G * T_Gr_Wr
            #print("Moving to pregrasp pose...")

            mv_time = self.__velma.getCartImpMvTime(self.__arm_side, dT_B_Wr_init, max_vel_lin, max_vel_rot)
            if not self.__velma.moveCartImp(self.__arm_side, [dT_B_Wr_init], [mv_time], None, None, None, None,
                    PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
                exitError(8)
            if self.__velma.waitForEffector(self.__arm_side) != 0:
                exitError(9)

            vs_cmd = Int32()
            if vs_mode == 'visual_servo':
                vs_cmd.data = 1
            else:
                vs_cmd.data = 2
            rospy.sleep(0.5)
            self.__vs_cmd_pub.publish(vs_cmd)
            rospy.sleep(0.5)

            self.publishInfo( 'begin approach' )

            print('Sending command to visual servo...')
            trj = CommandCartImpTrjPose()
            trj.count = 1
            trj.trj[0].pose = pm.toMsg( dT_B_Wr )
            trj.trj[0].time_from_start = rospy.Duration(3)
            self.__vs_trj_pub.publish(trj)
            rospy.sleep(3.0)
            print('The motion by visual servo should have finished')

        self.publishInfo( 'close fingers' )

        print("Closing fingers...")
        self.__velma.moveHand(self.__arm_side, hand_dest_q, [1, 1, 1, 1], force_limit, 1000, hold=False)
        if self.__velma.waitForHand(self.__arm_side) != 0:
            exitError(6)

        rospy.sleep(1)

        self.publishInfo( 'open fingers' )

        print("Opening fingers...")
        hand_q = [math.radians(0), math.radians(0), math.radians(0), 0]
        self.__velma.moveHand(self.__arm_side, hand_q, [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=False)
        if self.__velma.waitForHand(self.__arm_side) != 0:
            exitError(6)

        self.publishInfo( 'test finished' )

        if move_arm_back:
            print("Moving to the intermadiate position...")
            self.__velma.moveJoint(self.__q_map_2, None, max_vel=max_vel, start_time=0.5, position_tol=15.0/180.0*math.pi)
            error = self.__velma.waitForJoint()
            if error != 0:
                exitError(6, msg='The action moveJoint ended with error code: {}'.format(error))

        print('Done.')
        return 0

def main():
    rospack = rospkg.RosPack()
    path = rospack.get_path('visual_servo_end_effector') +\
                                                '/../../../experiments_visual_servo_end_effector/'

    if not os.path.isdir(path):
        print('ERROR: directory "{}" does not exist'.format(path))
        return 1

    test = VisualServoTests()

    test.makeTest('visual_servo', -0.01)
    test.writeData(path + 'test_0.txt')
    test.clearData()
    test.makeTest('visual_servo', 0.0)
    test.writeData(path + 'test_1.txt')
    test.clearData()
    test.makeTest('visual_servo', 0.01)
    test.writeData(path + 'test_2.txt')
    test.clearData()
    test.makeTest('visual_servo_manip_strategy', -0.01)
    test.writeData(path + 'test_3.txt')
    test.clearData()
    test.makeTest('visual_servo_manip_strategy', 0.0)
    test.writeData(path + 'test_4.txt')
    test.clearData()
    test.makeTest('visual_servo_manip_strategy', 0.01)
    test.writeData(path + 'test_5.txt')
    test.clearData()
    return 0

if __name__ == "__main__":
    exitError(main())

'''
Manipulation strategy - interpolation of stiffness for visual servo and grasping
Far from the object: high stiffness
Near the object: medium stiffness
At grasping pose (after start grasping event): low stiffness
After the grasp: set current equilibrium to the current pose and set stiffness to the desired value
'''
