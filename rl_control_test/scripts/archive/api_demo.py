#!/usr/bin/python3
import time
import rospy
from quadrotor_msgs.msg import RLControl
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,Imu
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from rl_control_test.srv import Sfc,SfcRequest,SfcResponse
from std_srvs.srv import Empty
import random
random.seed(42)

def odomCallback(odom_msg:Odometry):
    pass

def cloudCallback(cloud_msg:PointCloud2):
    #print(cloud_msg)
    pass

def imuCallBack(imu_msg:Imu):
    #print(imu_msg)
    pass

def getSo3cmd()->RLControl:
    command = RLControl()
    command.force = 0
    command.m1 = 0
    command.m2 = 0
    command.m3 = 0
    return command

def random_generate(sfc_srv):
    try:
        sfc_req = SfcRequest()
        sfc_res:SfcResponse = sfc_srv(sfc_req)
        print("success")
        print(sfc_res)
    except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))

def main():
    rospy.init_node("rl_test_py")
    so3cmd_pub = rospy.Publisher("/so3_cmd",RLControl,queue_size=1,tcp_nodelay=True)
    odom_sub = rospy.Subscriber("/quadrotor_odom",Odometry,odomCallback)
    imu_sub = rospy.Subscriber("/quadrotor_imu",Imu,imuCallBack)
    cloud_sub = rospy.Subscriber("/local_cloud",PointCloud2,cloudCallback)
    sfc_srv = rospy.ServiceProxy("/hpoly_srv",Sfc)
    step_srv = rospy.ServiceProxy("/quadrotor_simulator_so3/step",Empty)
    rospy.wait_for_service('/hpoly_srv')
    rospy.wait_for_service('/quadrotor_simulator_so3/step')
    rate = rospy.Rate(1)
    so3cmd_pub.publish(getSo3cmd())
    while not rospy.is_shutdown():
        #step_srv.call()
        random_generate(sfc_srv)
        rate.sleep()
if __name__=="__main__":
    main()