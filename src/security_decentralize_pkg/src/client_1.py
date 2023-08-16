#!/usr/bin/env python
import rospy
import os
# import matplotlib.pyplot as plt
# import numpy as np
from security_decentralize_pkg.msg import custom_1
from security_decentralize_pkg.msg import custom_2
from security_decentralize_pkg.msg import custom_3
from security_decentralize_pkg.msg import block1
from security_decentralize_pkg.msg import malicious_flags


import hashlib


t2=""
t3=""
t1=""
# x1=3
# x2=-1
# x3=3
# y1=3
# y2=-1
# y3=2
#
# plt.scatter(x1,y1,color='blue')
# plt.arrow(x1,y1,0.25,0,ec="green",head_width=0.1)
# x=np.array([x2,x3])
# y=np.array([y2,y3])
# plt.scatter(x,y,color='red')
# plt.arrow(x2,y2,-0.25,0,ec="green",head_width=0.1)
# plt.arrow(x3,y3,0,0.25,ec="green",head_width=0.1)
# plt.title("Robot 1")
# plt.grid()
# plt.xlim([-5,5])
# plt.ylim([-5,5])
# plt.show()

def create_hash_from_transaction(s):
    str1 = ""

    for ele in s:
        str1 += ele
    h=hashlib.sha256(str1.encode()).hexdigest()
    return h
    rospy.loginfo(str1)
    rospy.loginfo(hashlib.sha256(str1.encode()).hexdigest())

def talker():
    pub=rospy.Publisher('message_client1',custom_1,queue_size=1000)
    pub_hash=rospy.Publisher('client1_hash',block1,queue_size=1000)
    rospy.Subscriber('message_client2',custom_2,sub2)
    rospy.Subscriber('message_client3',custom_3,sub3)
    rospy.Subscriber('mal_bot_flag',malicious_flags,sub_flag)
    rospy.init_node('custom_talker_1',anonymous=True)

    r=rospy.Rate(1)

    msg=custom_1()
    msg2=block1()

    msg.robot_name.data="Robot_1"

    msg.location.x=2.0
    msg.location.y=3.0
    msg.location.theta=90.0

    while not rospy.is_shutdown():
        rospy.loginfo("robot1")
        rospy.loginfo(msg)
        pub.publish(msg)
        t1="x="+str(msg.location.x)+","+"y="+str(msg.location.y)+","+"w="+str(msg.location.theta)
        # rospy.loginfo(t1)
        # rospy.loginfo(t2)
        # rospy.loginfo(t3)
        hash1=create_hash_from_transaction([str(t1),str(t2),str(t3)])

        rospy.loginfo("hash=")
        rospy.loginfo(hash1)
        msg2.robot_hash.data=hash1
        pub_hash.publish(msg2)

        r.sleep()

def sub2(data2):
    global t2,x2,y2
    rospy.loginfo("robot2")
    rospy.loginfo(data2)
    t2="x="+str(data2.location.x)+","+"y="+str(data2.location.y)+","+"w="+str(data2.location.theta)


def sub3(data3):
    global t3,x3,y3
    rospy.loginfo("robot3")
    rospy.loginfo(data3)
    t3="x="+str(data3.location.x)+","+"y="+str(data3.location.y)+","+"w="+str(data3.location.theta)

def sub_flag(data_flag):
    flag="0"
    flag=data_flag.bot1_mal.data
    if(flag=="1"):
        rospy.loginfo("Malicious bot 1 killed")
        os._exit(0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
