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
                    js_msgs.append(js_msg)
                    # js_pub.publish(js_msg)
        #             rospy.sleep(0.04)
        #         if rospy.is_shutdown():
        #             break
        #     if rospy.is_shutdown():
        #         break
        # rospy.sleep(0.04)
    return js_msgs

def main():
    pass

if __name__ == "__main__":
    main()
