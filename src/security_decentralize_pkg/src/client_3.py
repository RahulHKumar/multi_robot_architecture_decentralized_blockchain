#!/usr/bin/env python
import rospy
import os
# import matplotlib.pyplot as plt
# import numpy as np
from security_decentralize_pkg.msg import custom_1
from security_decentralize_pkg.msg import custom_2
from security_decentralize_pkg.msg import custom_3
from security_decentralize_pkg.msg import block3
from security_decentralize_pkg.msg import malicious_flags


import hashlib

t1=""
t2=""
t3=""
# x1=2
# x2=-1
# x3=3
# y1=3
# y2=-1
# y3=2
#
# plt.scatter(x3,y3,color='blue')
# plt.arrow(x1,y1,0.25,0,ec="green",head_width=0.1)
# x=np.array([x2,x1])
# y=np.array([y2,y1])
# plt.scatter(x,y,color='red')
# plt.arrow(x2,y2,-0.25,0,ec="green",head_width=0.1)
# plt.arrow(x3,y3,0,0.25,ec="green",head_width=0.1)
# plt.title("Robot 3")
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
    pub=rospy.Publisher('message_client3',custom_3,queue_size=1000)
    pub_hash=rospy.Publisher('client3_hash',block3,queue_size=1000)
    rospy.Subscriber('message_client1',custom_1,sub1)
    rospy.Subscriber('message_client2',custom_2,sub2)
    rospy.Subscriber('mal_bot_flag',malicious_flags,sub_flag)
    rospy.init_node('custom_talker_3',anonymous=True)

    r=rospy.Rate(1)

    msg=custom_3()
    msg3=block3()

    msg.robot_name.data="Robot_3"

    msg.location.x=3.0
    msg.location.y=2.0
    msg.location.theta=0.0

    while not rospy.is_shutdown():
        # rospy.loginfo(msg)
        pub.publish(msg)
        t3="x="+str(msg.location.x)+","+"y="+str(msg.location.y)+","+"w="+str(msg.location.theta)
        # rospy.loginfo(t1)
        # rospy.loginfo(t2)
        # rospy.loginfo(t3)
        hash3=create_hash_from_transaction([str(t1),str(t2),str(t3)])

        rospy.loginfo("hash=")
        rospy.loginfo(hash3)
        msg3.robot_hash.data=hash3
        pub_hash.publish(msg3)

        r.sleep()

def sub1(data1):
    global t1
    # rospy.loginfo("robot1")
    # rospy.loginfo(data1)
    t1="x="+str(data1.location.x)+","+"y="+str(data1.location.y)+","+"w="+str(data1.location.theta)


def sub2(data2):
    global t2
    # rospy.loginfo("robot2")
    # rospy.loginfo(data2)
    t2="x="+str(data2.location.x)+","+"y="+str(data2.location.y)+","+"w="+str(data2.location.theta)

def sub_flag(data_flag):
    flag="0"
    flag=data_flag.bot3_mal.data
    if(flag=="1"):
        rospy.loginfo("Malicious bot 3 killed")
        os._exit(0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
