#!/usr/bin/python
#\file    bx_joint_states.py
#\brief   Convert /robot/joint_states topic to /joint_states for MoveIt with Baxter.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Dec.17, 2015
import roslib
import rospy
import sensor_msgs.msg
import copy

def Callback(pub_st, msg):
  remove_names= ('head_nod','torso_t0')
  if all(rmname not in msg.name for rmname in remove_names):
    pub_st.publish(msg)
  else:
    msg2= sensor_msgs.msg.JointState()
    msg2.header= msg.header
    idxs= [i for i,name in enumerate(msg.name) if name not in remove_names]
    msg2.name= [msg.name[i] for i in idxs]
    if len(msg.position)>0:  msg2.position= [msg.position[i] for i in idxs]
    if len(msg.velocity)>0:  msg2.velocity= [msg.velocity[i] for i in idxs]
    if len(msg.effort)>0:  msg2.effort= [msg.effort[i] for i in idxs]
    pub_st.publish(msg2)

if __name__=='__main__':
  rospy.init_node('bx_joint_states')
  pub_st= rospy.Publisher('/joint_states', sensor_msgs.msg.JointState, queue_size=1)
  sub_st= rospy.Subscriber('/robot/joint_states', sensor_msgs.msg.JointState, lambda msg: Callback(pub_st,msg))
  rospy.spin()
