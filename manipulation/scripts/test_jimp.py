#!/usr/bin/env python

## Runs test for jnt_imp mode motion.
# @ingroup integration_tests
# @file test_jimp.py
# @namespace scripts.test_jimp Integration test

# Copyright (c) 2017, Robot Control and Pattern Recognition Group,
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
import math
import PyKDL

from velma_common.velma_interface import VelmaInterface, isConfigurationClose,\
    symmetricalConfiguration
from control_msgs.msg import FollowJointTrajectoryResult
from rcprg_ros_utils import exitError

if __name__ == "__main__":
    # Define some configurations

    # every joint in position 0
    q_map_0 = symmetricalConfiguration( {'torso_0_joint':0,
        'right_arm_0_joint':0, 'right_arm_1_joint':0, 'right_arm_2_joint':0,
        'right_arm_3_joint':0, 'right_arm_4_joint':0, 'right_arm_5_joint':0,
        'right_arm_6_joint':0} )

    # starting position
    q_map_starting = symmetricalConfiguration( {'torso_0_joint':0,
        'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8, 'right_arm_2_joint':1.25,
        'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
        'right_arm_6_joint':0,} )

    # goal position
    q_map_goal = {'torso_0_joint':0,
        'right_arm_0_joint':0.3, 'right_arm_1_joint':1.8, 'right_arm_2_joint':-1.25,
        'right_arm_3_joint':2.0, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
        'right_arm_6_joint':0,
        'left_arm_0_joint':-0.3, 'left_arm_1_joint':-1.8, 'left_arm_2_joint':-1.25,
        'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5,
        'left_arm_6_joint':0 }

    # intermediate position
    # q_map_intermediate = {'torso_0_joint':0,
    #     'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.6, 'right_arm_2_joint':-1.25,
    #     'right_arm_3_joint':-0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
    #     'right_arm_6_joint':0,
    #     'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
    #     'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5,
    #     'left_arm_6_joint':0 }
    q_map_intermediate = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.6, 'right_arm_2_joint':-1.25,
        'right_arm_3_joint':-0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
        'right_arm_6_joint':0,
        'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
        'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5,
        'left_arm_6_joint':0 }
    
    # MY CONFIGURATIONS
    # pozycja pierwsza do ominięcia stolika przed chwyceniem
    q_map_my1 = {'torso_0_joint':-0.20245755137580862,
        'right_arm_0_joint':-0.040549221602448324, 'right_arm_1_joint':-1.939690065039993, 'right_arm_2_joint':2.20424097358419,
        'right_arm_3_joint':1.30317927455409, 'right_arm_4_joint':0.13618581972101784, 'right_arm_5_joint':-1.7205597887661666,
        'right_arm_6_joint':0.88517640014791}

    # pozycja przed złapaniem
    q_map_my2 = {'torso_0_joint':0.4963280397467873,
        'right_arm_0_joint':0.06471796460146129, 'right_arm_1_joint':-1.6963078134386786, 'right_arm_2_joint':2.7565891151139,
        'right_arm_3_joint':0.922547506515648, 'right_arm_4_joint':0.06210539575890758, 'right_arm_5_joint':-1.724952784130839,
        'right_arm_6_joint':0.7009604448356849}

    # pozycja z podniesioną szklanką
    q_map_my3 = {'torso_0_joint':0.49018119426368545,
        'right_arm_0_joint':-0.2134625908039628, 'right_arm_1_joint':-1.4318168216868312, 'right_arm_2_joint':2.316593354814221,
        'right_arm_3_joint':1.3099906386836455, 'right_arm_4_joint':0.541827047550822, 'right_arm_5_joint':-1.58827008358729,
        'right_arm_6_joint':0.35938230459677123}

    # pozycja podniesiona przed postawieniem
    q_map_my4 = {'torso_0_joint':0.8479370067212796,
        'right_arm_0_joint':0.48334617371245836, 'right_arm_1_joint':-1.4851888455833246, 'right_arm_2_joint':2.5712990063348475,
        'right_arm_3_joint':0.6432216068598393, 'right_arm_4_joint':0.020589908266191514, 'right_arm_5_joint':-1.6606441274634018,
        'right_arm_6_joint':1.1198170046419729}

    # pozycja tuz przed odstawieniam
    q_map_my5 = {'torso_0_joint':1.0233153675786633,
        'right_arm_0_joint':0.2431345866287985, 'right_arm_1_joint':-1.5587057734937435, 'right_arm_2_joint':2.7013997674575005,
        'right_arm_3_joint':0.658877630907585, 'right_arm_4_joint':0.009169826491319477, 'right_arm_5_joint':-1.7421822861349991,
        'right_arm_6_joint':1.0766135827339551}

    # pozycja po odstawieniu odsunieta
    q_map_my6 = {'torso_0_joint':0.8835198391548271,
        'right_arm_0_joint':0.14680695040294273, 'right_arm_1_joint':-1.6231777877950013, 'right_arm_2_joint':2.820061704034955,
        'right_arm_3_joint':0.7827351777338659, 'right_arm_4_joint':0.16126986744042832, 'right_arm_5_joint':-1.7425074959320823,
        'right_arm_6_joint':1.4308674622506423}

    rospy.init_node('test_jimp')

    rospy.sleep(0.5)

    print("This test/tutorial executes simple motions"\
        " in joint impedance mode. Planning is not used"\
        " in this example.")

    print("Running python interface for Velma...")
    velma = VelmaInterface()
    print("Waiting for VelmaInterface initialization...")
    if not velma.waitForInit(timeout_s=10.0):
        exitError(1, msg="Could not initialize VelmaInterface")
    print("Initialization ok!")

    print("Motors must be enabled every time after the robot enters safe state.")
    print("If the motors are already enabled, enabling them has no effect.")
    print("Enabling motors...")
    if velma.enableMotors() != 0:
        exitError(2, msg="Could not enable motors")

    rospy.sleep(0.5)

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        exitError(1, msg="Motors must be homed and ready to use for this test.")


    print("Switch to jnt_imp mode (no trajectory)...")
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        exitError(3, msg="The action should have ended without error,"\
                        " but the error code is {}".format(error))

    print("Checking if the starting configuration is as expected...")
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.3):
        exitError(10, msg="This test requires starting pose: {}".format(q_map_starting))

    print("Moving to position 0 (this motion is too fast and should cause error condition,"\
            " that leads to safe mode in velma_core_cs).")
    velma.moveJoint(q_map_0, 0.05, start_time=0.5, position_tol=0, velocity_tol=0)
    error = velma.waitForJoint()
    if error != FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
        exitError(4, msg="The action should have ended with PATH_TOLERANCE_VIOLATED"\
                    " error status, but the error code is {}".format(error) )

    print("Checking if current pose is close to the starting pose...")
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.3):
        exitError(10)

    print("waiting 2 seconds...")
    rospy.sleep(2)

    print("Motors must be enabled every time after the robot enters safe state.")
    print("If the motors are already enabled, enabling them has no effect.")
    print("Enabling motors...")
    if velma.enableMotors() != 0:
        exitError(5)

    print("Moving to position 0 (slowly).")
    velma.moveJoint(q_map_0, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_0, js[1], tolerance=0.1):
        exitError(10)

    rospy.sleep(1.0)

    print("Moving to the starting position...")
    velma.moveJoint(q_map_starting, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        exitError(6, msg="The action should have ended without error,"\
                        " but the error code is {}".format(error))

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.1):
        exitError(10)

    print("Moving to valid position, using invalid self-colliding trajectory"\
        " (this motion should cause error condition, that leads to safe mode in velma_core_cs).")
    velma.moveJoint(q_map_goal, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
        exitError(7, msg="The action should have ended with PATH_TOLERANCE_VIOLATED"\
                    " error status, but the error code is {}".format(error) )

    print("Using relax behavior to exit self collision...")
    velma.switchToRelaxBehavior()

    print("waiting 2 seconds...")
    rospy.sleep(2)

    print("Motors must be enabled every time after the robot enters safe state.")
    print("If the motors are already enabled, enabling them has no effect.")
    print("Enabling motors...")
    if velma.enableMotors() != 0:
        exitError(8)

    print("Moving to the starting position...")
    velma.moveJoint(q_map_starting, 4.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        exitError(9, msg="The action should have ended without error,"\
                        " but the error code is {}".format(error))

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.1):
        exitError(10)

    print("To reach the goal position, some trajectory must be exetuted that contains additional,"\
            " intermediate nodes")

    print("Moving to the intermediate position...")
    velma.moveJoint(q_map_intermediate, 8.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        exitError(10, msg="The action should have ended without error,"\
                        " but the error code is {}".format(error))

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_intermediate, js[1], tolerance=0.1):
        exitError(10)

    print("Moving to the goal position.")
    velma.moveJoint(q_map_goal, 8.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        exitError(11, msg="The action should have ended with PATH_TOLERANCE_VIOLATED"\
                        " error status, but the error code is {}".format(error) )

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_goal, js[1], tolerance=0.1):
        exitError(10)

    print("Moving to the intermediate position...")
    velma.moveJoint(q_map_intermediate, 8.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        exitError(12, msg="The action should have ended without error,"\
                        " but the error code is {}".format(error))

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_intermediate, js[1], tolerance=0.1):
        exitError(10)

    print("Moving to the starting position...")
    velma.moveJoint(q_map_starting, 5.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        exitError(13, msg="The action should have ended without error,"\
                        " but the error code is {}".format(error))

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.1):
        exitError(10)

    exitError(0)