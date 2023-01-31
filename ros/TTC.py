#!/usr/bin/env python3
import rospy
from mr_voice.msg import Voice
import cv2
import yaml


def callback_voice(msg):
    global _voice
    _voice = msg


def text_to_cmd(s):
    global _cmds 
    if _cmds is None or s is None: return None
    s = s.lower()
    for c in _cmds["QnA"]:
        OK = True 
        for i in c["K"]:
            tmp = False
            for j in i:
                if str(j).lower() in s:
                    tmp = True
                    break
            if not tmp: OK = False
        if OK: return c
    return None


if __name__  == "__main__":
    rospy.init_node("demo3")
    rospy.loginfo("demo3 start!")

    _cmds = None
    with open("/home/pcms/catkin_ws/src/beginner_tutorials/src/cmd.txt", "r") as f:
        try:
            _cmds = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print(e)
    
    _voice = None
    _topic_voice = "/voice/text"
    rospy.Subscriber(_topic_voice, Voice, callback_voice)

    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()

        if _voice is not None:
            rospy.loginfo("%s (%d)" % (_voice.text, _voice.direction))
            
            _cmd = text_to_cmd(_voice.text)
            if _cmd is not None:
                print(_cmd)
            
            _voice = None
            
    rospy.loginfo("demo3 end!")
    
