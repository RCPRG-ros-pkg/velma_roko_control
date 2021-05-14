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

import matplotlib.pyplot as plt

from rcprg_ros_utils import exitError

def plotData(path, filename):
    with open(path+filename, 'r') as f:
        lines = f.readlines()

    times = []
    forces = []
    torques = []
    strategy_begin = None
    strategy_end = None
    gripper_open_time = None
    gripper_close_time = None
    begin_time = None
    test_finished_time = None
    title = None
    for line in lines:
        fields = line.split(';')
        #print fields
        item_type = fields[0]
        item_time = float(fields[1])
        if item_type == 'ft':
            force = PyKDL.Vector( float(fields[2]), float(fields[3]), float(fields[4]) )
            torque = PyKDL.Vector( float(fields[5]), float(fields[6]), float(fields[7]) )

            #total_force = math.sqrt( fx**2 + fy**2 + fz**2 )
            #total_torque = math.sqrt( tx**2 + ty**2 + tz**2 )

            forces.append( force )
            torques.append( torque )
            times.append( item_time )

        elif item_type == 'strategy':
            strategy = int(fields[2])
            if strategy == 0:
                if strategy_begin is None:
                    strategy_begin = item_time
                strategy_end = item_time
        elif item_type == 'info':
            info = fields[2].strip()
            if title is None:
                title = info[12:]
            if info == 'close fingers':
                gripper_close_time = item_time
                grav_comp_force = forces[-1]
            elif info == 'open fingers':
                gripper_open_time = item_time
            elif info == 'begin approach':
                begin_time = item_time
            elif info == 'test finished':
                test_finished_time = item_time

            # TODO

    #width = 0.35
    #plt.rcParams["figure.figsize"] = (10,5)
    #plt.rcParams["figure.figsize"] = (5,2.5)
    #plt.bar(x - width/2, y_a, width, label='simulated')
    #plt.bar(x + width/2, y_b, width, label='hardware')
    forces_comp = [(force-grav_comp_force).Norm() for force in forces]
    torques_abs = [torque.Norm() for torque in torques]
    times_rel = [time - begin_time for time in times]
    plt.plot(times_rel, forces_comp, color='black', label='force')
    plt.plot(times_rel, torques_abs, color='green', label='torque')

    if not strategy_begin is None:
        #plt.vlines([strategy_begin, strategy_end], 7, 14, colors='r') 
        plt.vlines([strategy_begin-begin_time], 0, 5, colors='r')
        plt.annotate('commanded low stiffness', xy=(strategy_begin-begin_time, 5), xytext=(strategy_begin-begin_time+1, 5.7),
                 arrowprops=dict(facecolor='black', shrink=0.05),
                )

    plt.vlines([gripper_close_time-begin_time], 0, 5, colors='b')
    plt.annotate('close gripper', xy=(gripper_close_time-begin_time, 5), xytext=(gripper_close_time-begin_time+1, 5.4),
             arrowprops=dict(facecolor='black', shrink=0.05),
            )


    plt.vlines([gripper_open_time-begin_time], 0, 5, colors='b')
    plt.annotate('open gripper', xy=(gripper_open_time-begin_time, 5), xytext=(gripper_open_time-begin_time+1, 5.4),
             arrowprops=dict(facecolor='black', shrink=0.05),
            )

    plt.ylim(-0.5, 6)
    plt.xlim(0.0, 9.0)#test_finished_time-begin_time)
    #plt.xticks(x[0:-1]+0.5, ranges_str, rotation=90)
    plt.xlabel('time [s]')
    plt.ylabel('force [N], torque [Nm]')
    plt.legend(loc='upper left')
    plt.title(title)
    #plt.subplots_adjust(left=0.08, right=0.99, top=0.98, bottom=0.1)
    #plt.subplots_adjust(left=0.16, right=0.98, top=0.96, bottom=0.27)
    plt.show()

def main():
    rospack = rospkg.RosPack()
    path = rospack.get_path('visual_servo_end_effector') +\
                                                '/../../../experiments_visual_servo_end_effector/'

    if not os.path.isdir(path):
        print('ERROR: directory "{}" does not exist'.format(path))
        return 1

    filenames = []
    for i in range(6):
        filenames.append( 'test_{}.txt'.format(i) )

    for filename in filenames:
        plotData( path, filename )

    return 0

if __name__ == "__main__":
    exitError(main())
