import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
 
import rospy
import copy
import os
 
from velma_common import *
from rcprg_ros_utils import exitError

if __name__ == "__main__":
    #Cleanup
    os.system('rosrun velma_common reset_shm_comm.py')

    # Define some configurations
 
    # every joint in position 0
    q_torso_0 = symmetricalConfiguration( {'torso_0_joint':0} )

    q_torso_left = symmetricalConfiguration( {'torso_0_joint':1.55} )

    q_torso_right = symmetricalConfiguration( {'torso_0_joint':-1.55} )

    q_arms_up = symmetricalConfiguration( {'right_arm_0_joint':0, 'right_arm_1_joint':0, 
        'right_arm_2_joint':0, 'right_arm_3_joint':0, 'right_arm_4_joint':0,
        'right_arm_5_joint':0, 'right_arm_6_joint':0} )

    q_arms_down = symmetricalConfiguration( {'right_arm_0_joint':-2.96, 
        'right_arm_1_joint':-2.09, 'right_arm_2_joint':-2.96, 'right_arm_3_joint':-2.09,
        'right_arm_4_joint':-2.96, 'right_arm_5_joint':-2.09, 'right_arm_6_joint':-2.96} )

    #arms up
    # q_torso_left = symmetricalConfiguration( {'torso_0_joint':1.55,
    #     'right_arm_0_joint':0, 'right_arm_1_joint':0, 'right_arm_2_joint':0,
    #     'right_arm_3_joint':0, 'right_arm_4_joint':0, 'right_arm_5_joint':0,
    #     'right_arm_6_joint':0} )


    rospy.init_node('make_octomap', anonymous=False)
    rospy.sleep(0.5)

    print "This script executes simple motions"\
         " in order to create octomap."
 
    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    diag = velma.getCoreCsDiag()

    os.system('rosrun velma_task_cs_ros_interface initialize_robot.py')

    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)
    else:
        print "Motors homed and ready to use for this test."

    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(2)
    
    os.system('rosrun velma_task_cs_ros_interface reset_head_position.py')

    print "Moving to the current position..."
    js_start = velma.getLastJointState()
    velma.moveJoint(js_start[1], 0.5, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)


    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 1.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(4)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(5)

    print("moving torso to the left (max)")
    velma.moveJoint(q_torso_left, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    # I dont kwon how to chek it
    # if not isConfigurationClose(q_torso_left, js[0], tolerance=0.1):
    #     exitError(6)

    print "moving head to position: left (max)"
    q_dest = (1.56, 0)
    velma.moveHead(q_dest, 3.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(7)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(8)

    print "moving head to position: left down"
    q_dest = (1.56, 0.7)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(9)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(10)

    print "moving head to position: front down"
    q_dest = (0, 0.7)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(11)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(12)
    

    print("moving torso to the front")
    velma.moveJoint(q_torso_0, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    # I dont know how to check it
    # if not isConfigurationClose(q_torso_0, js[0], tolerance=0.1):
    #     exitError(13)

    print "moving head to position: front up"
    q_dest = (0, 0)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(14)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(15)

    print("moving torso to the right (max)")
    velma.moveJoint(q_torso_right, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    # I dont know how to check it
    # if not isConfigurationClose(q_torso_right, js[0], tolerance=0.1):
    #     exitError(16)

    print "moving head to position: right (max)"
    q_dest = (-1.56, 0)
    velma.moveHead(q_dest, 3.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(17)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
         exitError(18)

    print "moving head to position: right down"
    q_dest = (-1.56, 0.7)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(19)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(20)

    print "moving head to position: front down"
    q_dest = (0, 0.7)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(21)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(22)
    

    print("moving torso to the front")
    velma.moveJoint(q_torso_0, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    # I dont know how to check it
    # if not isConfigurationClose(q_torso_0, js[0], tolerance=0.1):
    #     exitError(23)

    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 1.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(4)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(24)


    print("moving arms up")
    velma.moveJoint(q_arms_up, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    # I dont know how to check it
    # if not isConfigurationClose(q_arms_up, js[0], tolerance=0.1):
    #     exitError(25)

    print("moving torso to the left (max)")
    velma.moveJoint(q_torso_left, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    # I dont kwon how to chek it
    # if not isConfigurationClose(q_torso_left, js[0], tolerance=0.1):
    #     exitError(26)

    print("moving torso to the right (max)")
    velma.moveJoint(q_torso_right, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    # I dont know how to check it
    # if not isConfigurationClose(q_torso_right, js[0], tolerance=0.1):
    #     exitError(27)

    print("moving torso to the front")
    velma.moveJoint(q_torso_0, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    # I dont know how to check it
    # if not isConfigurationClose(q_torso_0, js[0], tolerance=0.1):
    #     exitError(28)

    print("moving arms down")
    velma.moveJoint(q_arms_down, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    # I dont know how to check it
    # if not isConfigurationClose(q_arms_down, js[0], tolerance=0.1):
    #     exitError(29)

    exitError(0)