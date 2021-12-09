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
from visualization_msgs.msg import *
from sensor_msgs.msg import JointState 
from rcprg_ros_utils.marker_publisher import *
from velma_kinematics.velma_ik_geom import KinematicsSolverLWR4, KinematicsSolverVelma

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

# check if object is close enough to robot
def isCloseEnough(object_pose, robot_pose):
    s = math.sqrt((object_pose[0] - robot_pose[0]) ** 2 + (object_pose[1] - robot_pose[1]) ** 2 + (object_pose[2] - robot_pose[2]) ** 2)
    x = abs(object_pose[0])
    y = abs(object_pose[1])
    # z in range (0.6, 1.4)
    z = abs(object_pose[2])
    if s < 1.2 and x > 0.65 and y > 0.4 and z < 1.4 and z > 0.6:
        return True
    else:
        return False

def getIk(arm_name, T_A0_A7d, ampl):
    positions = []
    assert arm_name in ('right', 'left')
 
    m_pub = MarkerPublisher('velma_ik_geom')
    js_pub = rospy.Publisher('/joint_states', JointState, queue_size=1000)
    rospy.sleep(0.5)
 
    solv = KinematicsSolverLWR4()
 
    js_msg = JointState()
    for i in range(7):
        js_msg.name.append('{}_arm_{}_joint'.format(arm_name, i))
        js_msg.position.append(0.0)
 
    js_msg.name.append('torso_0_joint')
    js_msg.position.append(0.0)
 
    base_link_name = 'calib_{}_arm_base_link'.format(arm_name)
    phase = 0.0
    # while not rospy.is_shutdown():
    for index in range(100000):
        # print js_msg
        tx = ampl * math.sin(phase)
        ty = ampl * math.sin(phase*1.1)
        tz = ampl * math.sin(phase*1.2)
        T_A0_A7d2 = PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(tx, ty, tz)) * T_A0_A7d
        elbow_circle_angle = phase*1.3
        phase += 0.01
 
        flip_elbow = (math.sin(phase*1.51) > 0)
        flip_ee = (math.sin(phase*1.93) > 0)
        flip_shoulder = (math.sin(phase*2.73) > 0)
        q = solv.calculateIk(T_A0_A7d2, elbow_circle_angle, flip_shoulder, flip_elbow, flip_ee)
        pt_shoulder = solv.getDebug('pt_shoulder')
        elbow_pt = solv.getDebug('elbow_pt')
        ee_pt = solv.getDebug('ee_pt')
        #T_A0_A6 = solv.getDebug('T_A0_A6')
        m_id = 0
        if not pt_shoulder is None and not ee_pt is None:
            m_id = m_pub.publishVectorMarker(pt_shoulder, ee_pt, m_id, 1, 0, 0, a=1,
                                            frame=base_link_name, namespace='default', scale=0.01)
        if not pt_shoulder is None and not elbow_pt is None:
            m_id = m_pub.publishVectorMarker(pt_shoulder, elbow_pt, m_id, 1, 0, 0, a=1,
                                            frame=base_link_name, namespace='default', scale=0.01)
 
        m_id = m_pub.publishFrameMarker(T_A0_A7d2, m_id, scale=0.1,
                                                    frame=base_link_name, namespace='default')
        js_msg.header.stamp = rospy.Time.now()
        for i in range(7):
            if q[i] is None:
                js_msg.position[i] = 0.0
            else:
                js_msg.position[i] = q[i]
        # js_pub.publish(js_msg)
        positions.append(js_msg.position)
 
        # rospy.sleep(0.04)
    return positions

def maximumSum(state_list):
    return max(state_list, key = sum)

    # max_sum = sum(max(state_list, key = sum))
    # for state in state_list:
    #     if sum(state) == max_sum:
    #         return state

def goalPoint(table):
    goal_pos = [table.p[0], table.p[1], 1]
    return goal_pos

def getManipOrient(robot, goal):
    # if goal_pos[0] > robot_pos[0]:
    #     temp = goal_pos[0] > robot_pos[0]
    # else:
    #     temp = goal_pos[0] > robot_pos[0]
    my_z_rotation = math.atan2(goal.p[0] - robot.p[0], goal.p[1] - robot.p[1])
    print "my z rotation"
    print my_z_rotation
    # my_z_rotation = math.atan2(goal_pos[0], goal_pos[1])
    orig_base_rpy = robot.M.GetRPY()
    goal_orient_pickup = PyKDL.Rotation.RPY(orig_base_rpy[0], orig_base_rpy[1], my_z_rotation)
    return goal_orient_pickup

def create_state(position):
    print len(position), position
    q_new = {'torso_0_joint':position[0], 'right_arm_0_joint':position[1], 'right_arm_1_joint':position[2], 'right_arm_2_joint':position[3],
         'right_arm_3_joint':position[4], 'right_arm_4_joint':position[5], 'right_arm_5_joint':position[6], 'right_arm_6_joint':position[7],
         'left_arm_0_joint':0.4069, 'left_arm_1_joint':1.7526, 'left_arm_2_joint':-1.1731,
         'left_arm_3_joint':-0.8924, 'left_arm_4_joint':-0.4004, 'left_arm_5_joint':0.6363, 'left_arm_6_joint':0.1905}

    return q_new

def main():
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
    # T_B_Table_a = velma.getTf("B", "table_a")
    # T_B_Table_b = velma.getTf("B", "table_b")
    # T_B_Bowl = velma.getTf("B", "bowl")
    # T_B_torso = velma.getTf("B", "torso")
    T_B_Table_a = velma.getTf("B", "table_a")
    T_B_Table_b = velma.getTf("B", "table_b")
    T_B_Jar = velma.getTf("B", "jar")
    T_B_Bowl = velma.getTf("B", "bowl")
    T_B_torso = velma.getTf("B", "torso_link0")
    T_B_arm1 = velma.getTf("B", "right_arm_1_link")
    T_B_base = velma.getTf("B", "torso_base")

    # # test manip ortient
    # orient = getManipOrient(T_B_base, T_B_Jar)
    # print "orient"
    # print orient
    # T_B_Jar1 = T_B_Jar
    # T_B_Jar1.p[0] = 1
    # T_B_Jar1.p[1] = 1
    # print getManipOrient(T_B_base, T_B_Jar1)
    # T_B_Jar1.p[0] = -1
    # T_B_Jar1.p[1] = 1
    # print getManipOrient(T_B_base, T_B_Jar1)
    # T_B_Jar1.p[0] = 1
    # T_B_Jar1.p[1] = -1
    # print getManipOrient(T_B_base, T_B_Jar1)
    # T_B_Jar1.p[0] = -1
    # T_B_Jar1.p[1] = -1
    # print getManipOrient(T_B_base, T_B_Jar1)
    # return 0

    #Lookup where objects are located
    # print T_B_Table_a.M, T_B_Table_a.p
    # print T_B_Table_b.M, T_B_Table_b.p
    # print T_B_Jar.M, T_B_Jar.p
    # print T_B_Bowl.M, T_B_Bowl.p
    

    orig_orient = T_B_base.M
    orig_pose = [T_B_base.p[0], T_B_base.p[1], T_B_arm1.p[2]]
    orig_base_rpy = T_B_base.M.GetRPY()

    # check if can reach
    robot_pose = [T_B_base.p[0], T_B_base.p[1], T_B_arm1.p[2]]
    if isCloseEnough(T_B_Jar.p, robot_pose):
        print "OBJECT IN REACH, PROCEEDING!!!"
    else:
        print "OBJECT IS TOO FAR AWAY!!!"
        print "EXITING..."
        return 0

    # # manip orientation
    # manip_orient = getManipOrient(T_B_base, T_B_Jar)
    # tmp = PyKDL.Frame(manip_orient, T_B_Jar.p)
    # makeCimpMove(velma, tmp, "TEST MOOVE MANIP WITH ORIENT!!!")

    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()

    
    print "MOVING TO START POSITION!!!"
    planAndExecute(velma, p, q_start)


    print "MOVING TO INTERMEDIATE POSITION!!!"
    # planAndExecute(velma, p, q_pre_pickup)
    T_A0_A7d = PyKDL.Frame(T_B_Jar.M, PyKDL.Vector(T_B_Jar.p[0] - 0.3, T_B_Jar.p[1], T_B_Jar.p[2] + 0.1))
    states = getIk( 'right', T_B_Jar, 0.5 )
    state = maximumSum(states)
    print state
    # for state in states:
    #     planAndExecute(velma, p, create_state(state.position))
    planAndExecute(velma, p, create_state(state))



    #Grip offset
    # print "T_B_Jar"
    # print T_B_Jar
    # print "T_B_Jar.M"
    # print T_B_Jar.M
    # print "T_B_Jar.p"
    # print T_B_Jar.p

    # print "T_B_Table_a.M"
    # print T_B_Table_a.M
    # print "T_B_Table_a.p"
    # print T_B_Table_a.p

    T_B_Jar = PyKDL.Frame(T_B_Jar.M, PyKDL.Vector( T_B_Jar.p[0] - 0.25, T_B_Jar.p[1], T_B_Jar.p[2] + 0.1))
    # T_B_Jar = PyKDL.Frame(goal_orient_pickup, PyKDL.Vector( T_B_Jar.p[0] - 0.25, T_B_Jar.p[1], T_B_Jar.p[2] + 0.1))
    makeCimpMove(velma, T_B_Jar, "MOVING TOWARDS THE JAR!!!")


    # HERE SHOULD CLOSE GRIP
    print("CLOSING GRIP!!!")
    velma.moveHandRight([1.5, 1.5, 1.5, 0], [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=True)
    rospy.sleep(5)


    #Grip offset
    T_B_Jar_up = PyKDL.Frame(T_B_Jar.M, PyKDL.Vector( T_B_Jar.p[0], T_B_Jar.p[1], T_B_Jar.p[2] + 0.3))
    makeCimpMove(velma, T_B_Jar_up, "MOVING WITH AN OBJECT UPWARDS")


    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(4)
    
    
    print "MOVING JAR TO FINAL POSITION!!!"
    planAndExecute(velma, p, q_before_laydown)


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
    T_B_Wr_down = velma.getTf("B", "Wr")
    #Grip offset
    T_B_Wr_down = PyKDL.Frame(T_B_Wr_down.M, PyKDL.Vector( T_B_Wr_down.p[0], T_B_Wr_down.p[1], T_B_Wr_down.p[2] + 0.3))
    makeCimpMove(velma, T_B_Wr_down, "DISTANCING FROM THE JAR")


    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(4)
    
    print "MOVING TO START POSITION!!!"
    planAndExecute(velma, p, q_start)


    print "ENDING OPERATION!!!"

    rospy.sleep(1.0)
    return 0

if __name__ == "__main__":
    main()
