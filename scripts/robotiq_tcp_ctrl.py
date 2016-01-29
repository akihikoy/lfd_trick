#!/usr/bin/env python
'''
ROS node for controling a Robotiq C-Model gripper using the Modbus TCP protocol.
ref. swri-ros-pkg/robotiq/robotiq_c_model_control/nodes/CModelTcpNode.py
'''

import roslib
roslib.load_manifest('robotiq_c_model_control')
roslib.load_manifest('robotiq_modbus_tcp')
import rospy
import robotiq_c_model_control.baseCModel
import robotiq_modbus_tcp.comModbusTcp
import os, sys
import robotiq_c_model_control.msg as robotiq_msgs

def MainLoop(address, sleep_time):
  #Gripper is a C-Model with a TCP connection
  gripper= robotiq_c_model_control.baseCModel.robotiqBaseCModel()
  gripper.client= robotiq_modbus_tcp.comModbusTcp.communication()
  #We connect to the address received as an argument
  gripper.client.connectToDevice(address)

  #Topic of gripper status
  status_pub= rospy.Publisher('~status', robotiq_msgs.CModel_robot_input)
  #Topic of gripper command
  rospy.Subscriber('~command', robotiq_msgs.CModel_robot_output, gripper.refreshCommand)

  while not rospy.is_shutdown():
    #Get and publish the Gripper status
    status= gripper.getStatus()
    status_pub.publish(status)
    rospy.sleep(0.5*sleep_time)

    #Send the most recent command
    gripper.sendCommand()
    rospy.sleep(0.5*sleep_time)

if __name__ == '__main__':
  rospy.init_node('rq1')
  address= rospy.get_param('~address', 'rq1')
  freq= rospy.get_param('~freq', 400.0)

  MainLoop(address, 1.0/freq)
