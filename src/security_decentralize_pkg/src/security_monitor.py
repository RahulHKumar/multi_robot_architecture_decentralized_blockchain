#!/usr/bin/env python
import rospy
import hashlib
from security_decentralize_pkg.msg import block1
from security_decentralize_pkg.msg import block2
from security_decentralize_pkg.msg import block3
from security_decentralize_pkg.msg import malicious_flags

hash1=""
hash2=""
hash3=""

def most_frequent(list):
    counter=0
    num=list[0]
    for i in list:
        curr_frequency=list.count(i)
        if(curr_frequency>counter):
            counter=curr_frequency
            num=i
    return num

def check_percent_malicious(h1,h2,h3):
    list=[h1,h2,h3]
    maj=list.count(most_frequent(list))
    total=len(list)
    perc=float(maj)/total
    rospy.loginfo("percentage of malicious bots are")
    rospy.loginfo(perc)


def talker():
    rospy.Subscriber('client1_hash',block1,sub1)
    rospy.Subscriber('client2_hash',block2,sub2)
    rospy.Subscriber('client3_hash',block3,sub3)
    pub=rospy.Publisher('mal_bot_flag',malicious_flags,queue_size=1000)
    rospy.init_node('security_monitor',anonymous=True)

    flags=malicious_flags()

    r=rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.loginfo("hash1=")
        rospy.loginfo(hash1)
        rospy.loginfo("hash2=")
        rospy.loginfo(hash2)
        rospy.loginfo("hash3=")
        rospy.loginfo(hash3)
        flags.bot1_mal.data="0"
        flags.bot2_mal.data="0"
        flags.bot3_mal.data="0"

        if(hash1==hash2 and hash2==hash3 and hash3==hash1):
            rospy.loginfo("System secure")
        elif((hash1!=hash2 or hash1!=hash3) and hash2==hash3):
            rospy.loginfo("Bot 1 malicious!!!")
            flags.bot1_mal.data="1"
            if(hash2==hash3):
                rospy.loginfo("System fixed by killing bot 1")
        elif((hash1!=hash2 or hash2!=hash3) and hash1==hash3):
            rospy.loginfo("Bot 2 malicious!!!")
            flags.bot2_mal.data="1"
            if(hash1==hash3):
                rospy.loginfo("System fixed by killing bot 2")
        elif((hash3!=hash2 or hash1!=hash3) and hash1==hash2):
            rospy.loginfo("Bot 3 malicious!!!")
            flags.bot3_mal.data="1"
            if(hash1==hash2):
                rospy.loginfo("System fixed by killing bot 3")
        else:
            rospy.loginfo("Multiple bots have errors")
            check_percent_malicious(hash1,hash2,hash3)

        pub.publish(flags)
        r.sleep()


def sub1(data1):
    global hash1
    hash1=data1.robot_hash.data

def sub2(data2):
    global hash2
    hash2=data2.robot_hash.data

def sub3(data3):
    global hash3
    hash3=data3.robot_hash.data


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
