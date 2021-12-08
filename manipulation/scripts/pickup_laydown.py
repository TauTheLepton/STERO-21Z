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

    q_start = {'torso_0_joint':6.067961977395732e-07, 'right_arm_0_joint':-0.2892, 'right_arm_1_joint':-1.8185, 'right_arm_2_joint':1.2490,
         'right_arm_3_joint':0.8595, 'right_arm_4_joint':0.0126, 'right_arm_5_joint':-0.5617, 'right_arm_6_joint':0.0088,
         'left_arm_0_joint':0.4069, 'left_arm_1_joint':1.7526, 'left_arm_2_joint':-1.1731,
         'left_arm_3_joint':-0.8924, 'left_arm_4_joint':-0.4004, 'left_arm_5_joint':0.6363, 'left_arm_6_joint':0.1905}

    q_pre_pickup = {'torso_0_joint':-0.2024, 'right_arm_0_joint':-0.0405, 'right_arm_1_joint':-1.9396,
         'right_arm_2_joint':2.2042, 'right_arm_3_joint':1.3031, 'right_arm_4_joint':0.1361, 'right_arm_5_joint':-1.7205,
         'right_arm_6_joint':0.8851, 'left_arm_0_joint':0.4069, 'left_arm_1_joint':1.7526, 'left_arm_2_joint':-1.1731,
         'left_arm_3_joint':-0.8924, 'left_arm_4_joint':-0.4004, 'left_arm_5_joint':0.6363, 'left_arm_6_joint':0.1905 }
    
    #not needed anymore ?
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

    #Define a function for frequently used routine in this test
    def planAndExecute(q_dest):
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

    def makeCimpMove(Tf_frame, announcement):
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
        if not velma.moveCartImpRight([T_B_Jar], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(13)
        if velma.waitForEffectorRight() != 0:
            exitError(14)
        rospy.sleep(0.5)
        print "Calculating difference between desiread and reached pose..."
        T_B_T_diff = PyKDL.diff(Tf_frame, Tf_frame, 1.0)
        print T_B_T_diff
        if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
            exitError(15)


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


    #Get objects position form Gazebo
    T_B_Table_a = velma.getTf("B", "table_a")
    T_B_Table_b = velma.getTf("B", "table_b")
    T_B_Jar = velma.getTf("B", "jar")
    T_B_Bowl = velma.getTf("B", "bowl")
    #Lookup where objects are located
    # print T_B_Table_a.M, T_B_Table_a.p
    # print T_B_Table_b.M, T_B_Table_b.p
    # print T_B_Jar.M, T_B_Jar.p
    # print T_B_Bowl.M, T_B_Bowl.p

    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    
    print "MOVING TO START POSITION!!!"
    planAndExecute(q_start)

    print "MOVING TO INTERMEDIATE POSITION!!!"
    planAndExecute(q_pre_pickup)


    T_B_Jar = PyKDL.Frame(T_B_Jar.M, PyKDL.Vector( T_B_Jar.p[0] - 0.25, T_B_Jar.p[1], T_B_Jar.p[2] + 0.1))
    makeCimpMove(T_B_Jar, "MOVING TOWARDS THE JAR!!!")
    # print "Switch to cart_imp mode (no trajectory)..."
    # if not velma.moveCartImpRightCurrentPos(start_time=0.2):
    #     exitError(10)
    # if velma.waitForEffectorRight() != 0:
    #     exitError(11)
 
    # rospy.sleep(0.5)
 
    # diag = velma.getCoreCsDiag()
    # if not diag.inStateCartImp():
    #     exitError(12, msg="The core_cs should be in cart_imp state, but it is not")

    # print "MOVING TOWARDS THE JAR!!!"
    # #Grip offset
    # T_B_Jar = PyKDL.Frame(T_B_Jar.M, PyKDL.Vector( T_B_Jar.p[0] - 0.25, T_B_Jar.p[1], T_B_Jar.p[2] + 0.1))
    # if not velma.moveCartImpRight([T_B_Jar], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
    #     exitError(13)
    # if velma.waitForEffectorRight() != 0:
    #     exitError(14)
    # rospy.sleep(0.5)
    # print "Calculating difference between desiread and reached pose..."
    # T_B_T_diff = PyKDL.diff(T_B_Jar, T_B_Jar, 1.0)
    # print T_B_T_diff
    # if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
    #     exitError(15)

    # HERE SHOULD CLOSE GRIP
    print("CLOSING GRIP!!!")
    velma.moveHandRight([1.5, 1.5, 1.5, 0], [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=True)
    rospy.sleep(5)

    T_B_Jar_up = PyKDL.Frame(T_B_Jar.M, PyKDL.Vector( T_B_Jar.p[0], T_B_Jar.p[1], T_B_Jar.p[2] + 0.3))
    makeCimpMove(T_B_Jar_up, "MOVING WITH AN OBJECT UPWARDS")
    # print "MOVING WITH AN OBJECT UPWARDS"
    # #Grip offset
    # T_B_Jar_up = PyKDL.Frame(T_B_Jar.M, PyKDL.Vector( T_B_Jar.p[0], T_B_Jar.p[1], T_B_Jar.p[2] + 0.3))
    # if not velma.moveCartImpRight([T_B_Jar_up], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
    #     exitError(13)
    # if velma.waitForEffectorRight() != 0:
    #     exitError(14)
    # rospy.sleep(0.5)
    # print "Calculating difference between desiread and reached pose..."
    # T_B_T_diff = PyKDL.diff(T_B_Jar_up, T_B_Jar_up, 1.0)
    # print T_B_T_diff
    # if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
    #     exitError(15)


    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(4)
    
    print "MOVING JAR TO FINAL POSITION!!!"
    planAndExecute(q_before_laydown)

    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(10)
    if velma.waitForEffectorRight() != 0:
        exitError(11)
 
    rospy.sleep(0.5)
 
    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        exitError(12, msg="The core_cs should be in cart_imp state, but it is not")

    rospy.sleep(1)
    print "OPENING GRIP!!!"
    velma.moveHandRight([0, 0, 0, 0], [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=True)
    rospy.sleep(5)

    print "DISTANCING FROM THE JAR"
    #Grip offset
    T_B_Wr_down = velma.getTf("B", "Wr")
    T_B_Wr_down = PyKDL.Frame(T_B_Wr_down.M, PyKDL.Vector( T_B_Wr_down.p[0], T_B_Wr_down.p[1], T_B_Wr_down.p[2] + 0.3))
    if not velma.moveCartImpRight([T_B_Wr_down], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(13)
    if velma.waitForEffectorRight() != 0:
        exitError(14)
    rospy.sleep(0.5)
    print "Calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(T_B_Wr_down, T_B_Wr_down, 1.0)
    print T_B_T_diff
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
        exitError(15)

    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(4)
    
    print "MOVING TO START POSITION!!!"
    planAndExecute(q_start)


    print "ENDING OPERATION!!!"

    rospy.sleep(1.0)
