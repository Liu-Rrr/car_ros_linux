import rospy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose2D,Twist
import math
import tf
import time
from threading import Thread
ar_pose = Pose2D()
ar_pose_origin = AlvarMarkers()

def ar_listen():
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, get_ar_pose)
    rospy.spin()

def get_ar_pose(msg):
    global ar_pose,ar_pose_origin
    if(len(msg.markers)>0):
        # ar_track
        ar_pose.x=msg.markers[0].pose.pose.position.x
        ar_pose.y=msg.markers[0].pose.pose.position.y
        # ar_pose.z=msg.markers[0].pose.pose.position.z

        ar_pose_origin = msg.markers[0].pose.pose

rospy.init_node("getArRadius")

subscribeRos_thread3 = Thread(target=ar_listen)
subscribeRos_thread3.start()
time.sleep(3)
while(1):
    odom_or= tf.transformations.euler_from_quaternion([0,0, ar_pose_origin.orientation.y, ar_pose_origin.orientation.z, ar_pose_origin.orientation.w])
    current_th=odom_or[2]
    # print(math.degrees(current_th))
    # print(abs(math.degrees(current_th))-90)
    print(abs(math.radians(abs(math.degrees(current_th))-90)))
    time.sleep(1)