#!/usr/bin/python
#\file    bx_set_gripw.py
#\brief   Set a virtual mass on a Baxter gripper.  Use this for a heavy gripper like Robotiq.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Dec.22, 2015
#cf. http://sdk.rethinkrobotics.com/wiki/Gripper_Customization
import roslib
import rospy
import baxter_core_msgs.msg
import time

if __name__=='__main__':
  rospy.init_node('bx_set_gripw')
  arm= rospy.get_param('~arm', 'left')  #left or right
  mass= rospy.get_param('~mass', 0.987)  #weight in kg
  gid= rospy.get_param('~gid', 131073)  #gripper id
  pub= rospy.Publisher('/robot/end_effector/{arm}_gripper/command'.format(arm=arm), baxter_core_msgs.msg.EndEffectorCommand, latch=True)

  cmd= baxter_core_msgs.msg.EndEffectorCommand()
  cmd.id= gid
  cmd.command= cmd.CMD_CONFIGURE
  cmd.args= '{{ "urdf":{{ "name": "{arm}_custom_gripper", "link": [ {{ "name": "{arm}_hand" }}, {{ "name": "{arm}_gripper_base", "inertial": {{ "mass": {{ "value": {mass} }}, "origin": {{ "xyz": [0.0, 0.0, 0.0] }} }} }} ] }} }}'.format(arm=arm, mass=mass)
  print 'Publishing command in 3 sec:'
  print cmd
  pub.publish(cmd)
  #t_start= time.time()
  #while time.time()-t_start<3.0:
    #pub.publish(cmd)
  rospy.sleep(3.0)
