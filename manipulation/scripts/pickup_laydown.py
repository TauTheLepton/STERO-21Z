import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
 
import rospy
import math
import PyKDL
 
from velma_common.velma_interface import VelmaInterface, isConfigurationClose,\
    symmetricalConfiguration
from velma_common import *
from control_msgs.msg import FollowJointTrajectoryResult
from rcprg_planner import *
from rcprg_ros_utils import exitError

if __name__ == "__main__":
    #Define some configurations

    #Starting position
    q_map_starting = {'torso_0_joint':0}

    q_start = {'torso_0_joint':6.067961977395732e-07, 'right_arm_0_joint':-0.2892, 'right_arm_1_joint':-1.8185, 'right_arm_2_joint':1.2490,
         'right_arm_3_joint':0.8595, 'right_arm_4_joint':0.0126, 'right_arm_5_joint':-0.5617, 'right_arm_6_joint':0.0088,
         'left_arm_0_joint':0.4069, 'left_arm_1_joint':1.7526, 'left_arm_2_joint':-1.1731,
         'left_arm_3_joint':-0.8924, 'left_arm_4_joint':-0.4004, 'left_arm_5_joint':0.6363, 'left_arm_6_joint':0.1905}

    q_pre_pickup = {'torso_0_joint':-0.2024, 'right_arm_0_joint':-0.0405, 'right_arm_1_joint':-1.9396,
         'right_arm_2_joint':2.2042, 'right_arm_3_joint':1.3031, 'right_arm_4_joint':0.1361, 'right_arm_5_joint':-1.7205,
         'right_arm_6_joint':0.8851, 'left_arm_0_joint':0.4069, 'left_arm_1_joint':1.7526, 'left_arm_2_joint':-1.1731,
         'left_arm_3_joint':-0.8924, 'left_arm_4_joint':-0.4004, 'left_arm_5_joint':0.6363, 'left_arm_6_joint':0.1905 }
    
    q_next_to_jar = {'torso_0_joint':0.4963280397467873,
        'right_arm_0_joint':0.06471796460146129, 'right_arm_1_joint':-1.6963078134386786, 'right_arm_2_joint':2.7565891151139,
        'right_arm_3_joint':0.922547506515648, 'right_arm_4_joint':0.06210539575890758, 'right_arm_5_joint':-1.724952784130839,
        'right_arm_6_joint':0.7009604448356849, 'left_arm_0_joint':0.4069, 'left_arm_1_joint':1.7526, 'left_arm_2_joint':-1.1731,
         'left_arm_3_joint':-0.8924, 'left_arm_4_joint':-0.4004, 'left_arm_5_joint':0.6363, 'left_arm_6_joint':0.1905}

    # pozycja podniesiona przed postawieniem
    q_before_laydown = {'torso_0_joint':1.0233153675786633,
        'right_arm_0_joint':0.2431345866287985, 'right_arm_1_joint':-1.5587057734937435, 'right_arm_2_joint':2.7013997674575005,
        'right_arm_3_joint':0.658877630907585, 'right_arm_4_joint':0.009169826491319477, 'right_arm_5_joint':-1.7421822861349991,
        'right_arm_6_joint':1.0766135827339551, 'left_arm_0_joint':0.4069, 'left_arm_1_joint':1.7526, 'left_arm_2_joint':-1.1731,
         'left_arm_3_joint':-0.8924, 'left_arm_4_joint':-0.4004, 'left_arm_5_joint':0.6363, 'left_arm_6_joint':0.1905}



    rospy.init_node('pickup_laydown')
    rospy.sleep(0.5)

    print "This test/tutorial executes complex motions"\
        " in Joint Impedance mode. Planning is used"\
        " in this example.\n"
 
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

    # velma = VelmaInterface()
    # velma.waitForInit()
    # p = Planner(velma.maxJointTrajLen())
    # p.waitForInit(): done 

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

    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    # if not isConfigurationClose(q_map_starting, js[1], tolerance=0.2):
    #     print "This test requires starting pose:"
    #     print q_map_starting
    #     exitError(10)

    print "Planning motion to the pre_pickup position using set of all joints..."

    # js = velma.getLastJointState()
    # if not isConfigurationClose(q_start, js[1]):
    print "MOVING TO START POSITION!!!"
    pre_pickup_constraint_1 = qMapToConstraints(q_start, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(15):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = p.plan(js[1], [pre_pickup_constraint_1], "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
            exitError(5)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_start, js[1]):
        exitError(6)

    # print "Moving to valid position, using planned trajectory."
    # pre_pickup_constraint_1 = qMapToConstraints(q_pre_pickup, 0.01, group=velma.getJointGroup("impedance_joints"))
    # for i in range(15):
    #     rospy.sleep(0.5)
    #     js = velma.getLastJointState()
    #     print "Planning (try", i, ")..."
    #     traj = p.plan(js[1], [pre_pickup_constraint_1], "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
    #     if traj == None:
    #         continue
    #     print "Executing trajectory..."
    #     if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
    #         exitError(5)
    #     if velma.waitForJoint() == 0:
    #         break
    #     else:
    #         print "The trajectory could not be completed, retrying..."
    #         continue

    # rospy.sleep(0.5)
    # js = velma.getLastJointState()
    # if not isConfigurationClose(q_pre_pickup, js[1]):
    #     exitError(6)

    print "MOVING TO THE JAR!!!"
    pre_pickup_constraint_2 = qMapToConstraints(q_next_to_jar, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(15):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = p.plan(js[1], [pre_pickup_constraint_2], "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
            exitError(5)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_next_to_jar, js[1]):
        exitError(6)

    # HERE SHOULD CLOSE GRIP
    print("CLOSING GRIP!!!")
    velma.moveHandRight([1.5, 1.5, 1.5, 0], [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=True)
    rospy.sleep(5)

    print "MOVING JAR TO FINAL POSITION!!!"
    post_pickup_constraint_1 = qMapToConstraints(q_before_laydown, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(15):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = p.plan(js[1], [post_pickup_constraint_1], "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
            exitError(5)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_before_laydown, js[1]):
        exitError(6)

    rospy.sleep(1)
    print "OPENING GRIP!!!"
    velma.moveHandRight([0, 0, 0, 0], [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=True)
    rospy.sleep(5)

    print "MOVING TO START POSITION!!!"
    post_pickup_constraint_2 = qMapToConstraints(q_start, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(15):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = p.plan(js[1], [post_pickup_constraint_2], "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
            exitError(5)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_start, js[1]):
        exitError(6)

    print "ENDING OPERATION!!!"

    rospy.sleep(1.0)
