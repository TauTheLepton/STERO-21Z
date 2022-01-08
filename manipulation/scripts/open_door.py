import rospy
import copy
import cv2
import PyKDL
import math
import numpy as np
import random

from visualization_msgs.msg import *
from sensor_msgs.msg import JointState

from rcprg_ros_utils.marker_publisher import *
from velma_kinematics.velma_ik_geom import KinematicsSolverLWR4, KinematicsSolverVelma

from velma_common.velma_interface import VelmaInterface, isConfigurationClose,\
    symmetricalConfiguration
from velma_common import *
from control_msgs.msg import FollowJointTrajectoryResult
from rcprg_planner import *
from rcprg_ros_utils import exitError

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')


#Define a function for frequently used routine in this test
def planAndExecute(velma, p, q_dest):
    print "Planning motion to the goal position using set of all joints..."
    print "Moving to valid position, using planned trajectory."
    goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(5):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5):
            exitError(5)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_dest, js[1]):
        exitError(6)

def makeCimpMove(velma, Tf_frame, announcement):
    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(10)
    if velma.waitForEffectorRight() != 0:
        exitError(11)

    rospy.sleep(0.5)

    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        exitError(12, msg="The core_cs should be in cart_imp state, but it is not")

    print announcement
    if not velma.moveCartImpRight([Tf_frame], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(13)
    if velma.waitForEffectorRight() != 0:
        exitError(14)
    rospy.sleep(0.5)
    print "Calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(Tf_frame, Tf_frame, 1.0)
    print T_B_T_diff
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
        exitError(15)

def randomOrientation():
    while True:
        qx = random.gauss(0.0, 1.0)
        qy = random.gauss(0.0, 1.0)
        qz = random.gauss(0.0, 1.0)
        qw = random.gauss(0.0, 1.0)
        q_len = qx**2 + qy**2 + qz**2 + qw**2
        if q_len > 0.001:
            qx /= q_len
            qy /= q_len
            qz /= q_len
            qw /= q_len
            return PyKDL.Rotation.Quaternion(qx, qy, qz, qw)

def getIk(arm_name, T_B_A7d, loop_max):
    """
    arm_name: 'right' or 'left'
    T_B_A7d: PyKDL.Frame with end effector desired position and orientation
    loop_max: how many times should try to get inverted kinematics
    """
    assert arm_name in ('right', 'left')
 
    # m_pub = MarkerPublisher('velma_ik_geom')
    # js_pub = rospy.Publisher('/joint_states', JointState, queue_size=1000)
    # rospy.sleep(0.5)
 
    flips = []
    for flip_shoulder in (True, False):
        for flip_elbow in (True, False):
            for flip_ee in (True, False):
                flips.append( (flip_shoulder, flip_elbow, flip_ee) )
    solv = KinematicsSolverVelma()

    # TODO: ADD VARIABLE TORSO ANGLE (I think?)
    torso_angle = 0.0

    if arm_name == 'right':
        central_point = PyKDL.Vector( 0.7, -0.7, 1.4 )
    else:
        central_point = PyKDL.Vector( 0.7, 0.7, 1.4 )

    js_msg = JointState()
    for i in range(7):
        js_msg.name.append('{}_arm_{}_joint'.format(arm_name, i))
        js_msg.position.append(0.0)

    js_msg.name.append('torso_0_joint')
    js_msg.position.append(torso_angle)

    base_link_name = 'calib_{}_arm_base_link'.format(arm_name)
    phase = 0.0
    js_msgs = []
    for index in range(loop_max):
        # # Get random pose
        # T_B_A7d = PyKDL.Frame(randomOrientation(), central_point + PyKDL.Vector(random.uniform(-0.4, 0.4), random.uniform(-0.4, 0.4), random.uniform(-0.4, 0.4)))
 
        # m_id = 0
        # m_id = m_pub.publishFrameMarker(T_B_A7d, m_id, scale=0.1, frame='world', namespace='default')
         
        for flip_shoulder, flip_elbow, flip_ee in flips:
            for elbow_circle_angle in np.linspace(-math.pi, math.pi, 20):
                arm_q = solv.calculateIkArm(arm_name, T_B_A7d, torso_angle, elbow_circle_angle, flip_shoulder, flip_elbow, flip_ee)
 
                if not arm_q[0] is None:
                    js_msg.header.stamp = rospy.Time.now()
                    for i in range(7):
                        js_msg.position[i] = arm_q[i]
                    # js_msgs.append(js_msg) TODO usunac?
                    return js_msg
                    # js_pub.publish(js_msg)
        #             rospy.sleep(0.04)
        #         if rospy.is_shutdown():
        #             break
        #     if rospy.is_shutdown():
        #         break
        # rospy.sleep(0.04)
    # return js_msgs TODO usunac?

def createState(state_msg, arm_name):
    assert arm_name in ('left', 'right')
    other_arm_name = 'left' if arm_name == 'right' else 'right'
    names = ['{}_arm_{}_joint'.format(other_arm_name, i) for i in range(7)]
    positions = [0.4069, 1.7526, -1.1731, -0.8924, -0.4004, 0.6363, 0.1905]
    state = {name: position for name, position in zip(state_msg.name + names, state_msg.position + positions)}
    return state

def closeGrip(velma):
    print "CLOSING GRIP!!!"
    velma.moveHandRight([1.5, 1.5, 1.5, 0], [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=True)
    rospy.sleep(5)

def openGrip(velma):
    print "OPENING GRIP!!!"
    velma.moveHandRight([0, 0, 0, 0], [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=True)
    rospy.sleep(5)

def halfCloseGrip(velma):
    print "HALF CLOSING GRIP!!!"
    velma.moveHandRight([0.75, 0.75, 0.75, 0], [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=True)
    rospy.sleep(5)

def myMessage(message):
    print "[MY MESSAGE]", message

def main():
    q_start = {'torso_0_joint':6.067961977395732e-07, 'right_arm_0_joint':-0.2892, 'right_arm_1_joint':-1.8185, 'right_arm_2_joint':1.2490,
         'right_arm_3_joint':0.8595, 'right_arm_4_joint':0.0126, 'right_arm_5_joint':-0.5617, 'right_arm_6_joint':0.0088,
         'left_arm_0_joint':0.4069, 'left_arm_1_joint':1.7526, 'left_arm_2_joint':-1.1731,
         'left_arm_3_joint':-0.8924, 'left_arm_4_joint':-0.4004, 'left_arm_5_joint':0.6363, 'left_arm_6_joint':0.1905}


    rospy.init_node('pickup_laydown')
    rospy.sleep(0.5)
    velma = VelmaInterface()

    print "This test/tutorial executes pickup and laydown task"\
        " In this example planning is used"\
 
    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        exitError(1, msg="Could not initialize VelmaInterface")
    print "Initialization ok!\n"

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        exitError(1, msg="Motors must be homed and ready to use for this test.")
 
    print "Waiting for Planner initialization..."
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit(timeout_s=10.0):
        exitError(2, msg="Could not initialize Planner")
    print "Planner initialization ok!"


    #Loading octomap
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)

    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(3)

    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(4)


    T_B_right_handle = velma.getTf("B", "right_handle")
    T_B_right_door = velma.getTf("B", "right_door")
    T_B_base = velma.getTf("B", "torso_base")
    # print "TORSO BASE POSITION"
    # print T_B_base

    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()

    # myMessage('MOVING TO START POSITION')
    # planAndExecute(velma, p, q_start)

    myMessage('MOVING TO INTERMEDIATE POSITION')
    # planAndExecute(velma, p, q_pre_pickup)
    orient = T_B_base.M.GetRPY()
    myMessage("MY RPY")
    myMessage(orient)

    T_A0_A7d = PyKDL.Frame(T_B_base.M, PyKDL.Vector(T_B_right_handle.p[0] - 0.35, T_B_right_handle.p[1] - 0.2, T_B_right_handle.p[2]))
    # print 'RIGHT HANDLE POSITION'
    # print T_B_right_handle
    state_msg = getIk( 'right', T_A0_A7d, 5 )
    if state_msg is None:
        myMessage('DID NOT GET INVERSED KINEMATICS')
        return 0
    # state = max(states, key=sum)
    myMessage('state msg')
    myMessage(state_msg)
    state = createState(state_msg, 'right')
    myMessage('state')
    myMessage(state)
    # for state in states:
    #     planAndExecute(velma, p, create_state(state.position))
    planAndExecute(velma, p, state)

    T_B_pre_right_handle = PyKDL.Frame(T_B_right_handle.M, PyKDL.Vector(T_B_right_handle.p[0] - 0.1, T_B_right_handle.p[1]-0.1, T_B_right_handle.p[2] + 0.05))
    makeCimpMove(velma, T_B_pre_right_handle, "MOVING TOWARDS THE HANDLE!!!")

    planAndExecute(velma, p, q_start)


if __name__ == "__main__":
    main()
