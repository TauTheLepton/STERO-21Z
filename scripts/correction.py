import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import Quaternion
# from quaternions import Quaternion
from geometry_msgs.msg._Quaternion import Quaternion

# declarations
callback_data_x = 0.0
callback_data_y = 0.0
callback_data_orient = 0.0

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

# corrects position and orientation - adds offset of the first position
def correction(mode):
    global callback_data_x, callback_data_y, callback_data_orient
    pub = rospy.Publisher('/mobile_base_controller/odom1', Odometry, queue_size=10)
    sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, odom_callback)
    rospy.init_node('correction', anonymous=True)
    rate = rospy.Rate(10) # 10hz 0.1s
    odom_msg = Odometry()
    while not rospy.is_shutdown():
        # low speed
        if mode == 1:
            callback_data_x += 0.162
            callback_data_y += 1.181
            callback_data_orient += 1.414
        # high speed
        elif mode == 2:
            callback_data_x += 0.308
            callback_data_y += 1.271
            callback_data_orient += 1.266
        # very high sped
        elif mode == 3:
            callback_data_x += 0.231
            callback_data_y += 1.225
            callback_data_orient += 1.297
        # normalize angle
        if callback_data_orient > math.pi:
            callback_data_orient = math.pi - (callback_data_orient - math.pi)
            callback_data_orient = -callback_data_orient
        q = quaternion_from_euler(0, 0, callback_data_orient)
        return_callback_data_orient = Quaternion(*q)
        # platynowy debugger
        # print(2)
        # print(type(return_callback_data_orient))
        odom_msg.pose.pose.position.x = callback_data_x
        odom_msg.pose.pose.position.y = callback_data_y
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation = return_callback_data_orient
        pub.publish(odom_msg)
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
            if mode < 1 or mode > 3:
                print("Choose mode '1' or '2' or '3'")
            else:
                incorrect_input = False
    try:
        correction(mode)
    except rospy.ROSInterruptException:
        pass
