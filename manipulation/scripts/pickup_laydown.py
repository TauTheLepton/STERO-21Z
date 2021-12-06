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

    q_pre_pickup = {'torso_0_joint':-0.2024, 'right_arm_0_joint':-0.0405, 'right_arm_1_joint':-1.9396,
         'right_arm_2_joint':2.2042, 'right_arm_3_joint':1.3031, 'right_arm_4_joint':0.1361, 'right_arm_5_joint':-1.7205,
         'right_arm_6_joint':0.8851, 'left_arm_0_joint':0.4069, 'left_arm_1_joint':1.7526, 'left_arm_2_joint':-1.1731,
         'left_arm_3_joint':-0.8924, 'left_arm_4_joint':-0.4004, 'left_arm_5_joint':0.6363, 'left_arm_6_joint':0.1905 }



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
    oml = OctomapListener("../data/new_octmap.bt")
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
    
    print "Moving to valid position, using planned trajectory."
    pre_pickup_constraint_1 = qMapToConstraints(q_pre_pickup, 0.01, group=velma.getJointGroup("impedance_joints"))
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
    if not isConfigurationClose(q_pre_pickup, js[1]):
        exitError(6)
 
    rospy.sleep(1.0)


