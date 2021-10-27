import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
# from gazebo_msgs import ModelStates
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import Quaternion
# from quaternions import Quaternion
from geometry_msgs.msg._Quaternion import Quaternion

"""
added offset to 'odom' data so it starts from 'map' position:
/mobile_base_controller/odom/pose/pose/position -> /mobile_base_controller/odom1/pose/pose/position
(/x, /y and /orientation (last one as quaternion))

combined X and Y axis speeds to one speed:
 gazebo/model_states/twist[1]/linear-> /combined/linear/x

quaternions to rpy:
/mobile_base_controller/odom/pose/pose/orientation -> /combined/angular/x
/gazebo/model_states/pose[1]/orientation -> /combined/angular/y

errors:
error in this one iteration -> /combined/linear/y
all errors summed -> /combined/linear/z
"""

# declarations
callback_data_x = 0.0
callback_data_y = 0.0
callback_data_orient = 0.0
vel_lin = 0.0
callback_data_orient_ref = 0.0
callback_data_x_ref = 0.0
callback_data_y_ref = 0.0
err = 0.0
err_sum = 0.0

# used in odometry subscriber
def odom_callback(odometry):
    global callback_data_x, callback_data_y, callback_data_orient
    callback_data_x = odometry.pose.pose.position.x
    callback_data_y = odometry.pose.pose.position.y
    orient = odometry.pose.pose.orientation
    # platynowy debugger
    # print(1)
    # print(type(orient))
    callback_data_orient = euler_from_quaternion((orient.x, orient.y, orient.z, orient.w))[2]

def odom_callback_vel(model_state):
    global vel_lin, callback_data_orient_ref
    vel_linX = model_state.twist[1].linear.x
    vel_linY = model_state.twist[1].linear.y
    orient = model_state.pose[1].orientation
    callback_data_orient_ref = euler_from_quaternion((orient.x, orient.y, orient.z, orient.w))[2]
    # platynowy debugger
    # print('----------')
    # print(vel_linX, vel_linY)
    vel_lin = math.sqrt(vel_linX ** 2 + vel_linY ** 2)

def global_pose_callback(data):
    global callback_data_x_ref, callback_data_y_ref, callback_data_x, callback_data_y, err, err_sum
    callback_data_x_ref = data.pose.pose.position.x
    callback_data_y_ref = data.pose.pose.position.y
    err = 0
    # variant A
    err = math.sqrt((callback_data_x - callback_data_x_ref) ** 2 + (callback_data_y - callback_data_y_ref) ** 2)
    # variant B
    # err = abs(callback_data_x - callback_data_x_ref) + abs(callback_data_y - callback_data_y_ref)
    # sum of errors
    err_sum += err
    # platynowy debugger
    # print(err)
    # print(err_sum)
    # print('-----')


# corrects position and orientation - adds offset of the first position
def correction(mode):
    global callback_data_x, callback_data_y, callback_data_orient, vel_lin, err, err_sum
    pub = rospy.Publisher('/mobile_base_controller/odom1', Odometry, queue_size=10)
    sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, odom_callback)
    pub_twist = rospy.Publisher('/combined', Twist, queue_size=10)
    sub_twist = rospy.Subscriber('/gazebo/model_states', ModelStates, odom_callback_vel)
    sub_global = rospy.Subscriber('/robot_pose', PoseWithCovarianceStamped, global_pose_callback)
    rospy.init_node('correction', anonymous=True)
    rate = rospy.Rate(10) # 10hz 0.1s
    odom_msg = Odometry()
    twist_msg = Twist()
    callback_data_x_adj = 0.0
    callback_data_y_adj = 0.0
    while not rospy.is_shutdown():
        # low speed
        if mode == 1:
            callback_data_x_adj = callback_data_x + 0.162
            callback_data_y_adj = callback_data_y + 1.181
            callback_data_orient += 1.414
        # high speed
        elif mode == 2:
            callback_data_x_adj = callback_data_x + 0.308
            callback_data_y_adj = callback_data_y + 1.271
            callback_data_orient += 1.266
        # very high sped
        elif mode == 3:
            callback_data_x_adj = callback_data_x + 0.231
            callback_data_y_adj = callback_data_y + 1.225
            callback_data_orient += 1.297
        else:
            pass
        # normalize angle
        if callback_data_orient > math.pi:
            callback_data_orient = math.pi - (callback_data_orient - math.pi)
            callback_data_orient = -callback_data_orient

        q = quaternion_from_euler(0, 0, callback_data_orient) 
        return_callback_data_orient = Quaternion(*q)
        # platynowy debugger
        # print(2)
        # print(type(return_callback_data_orient))
        odom_msg.pose.pose.position.x = callback_data_x_adj
        odom_msg.pose.pose.position.y = callback_data_y_adj
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation = return_callback_data_orient
        pub.publish(odom_msg)

        twist_msg.linear.x = vel_lin
        twist_msg.angular.x = callback_data_orient
        twist_msg.angular.y = callback_data_orient_ref
        pub_twist.publish(twist_msg)

        twist_msg.linear.y = err
        twist_msg.linear.z = err_sum

        rate.sleep()

if __name__ == '__main__':
    incorrect_input = True
    print("Choose mode '1' or '2' or '3'")
    while incorrect_input:
        mode = raw_input("Mode: ")
        try:
            mode = int(mode)
        except:
            print("Choose mode '1' or '2' or '3'")
        else:
            if mode < 1 or mode > 4:
                print("Choose mode '1' or '2' or '3'")
            else:
                incorrect_input = False
    try:
        correction(mode)
    except rospy.ROSInterruptException:
        pass
