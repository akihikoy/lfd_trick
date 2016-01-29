#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test pose estimation for gripper with ray tracing (3).
  Usage: test_poseestg
  '''
def Run(t,*args):
  gripper= 'wrist_l'
  sensor_type= 'ext'
  #Update attribute information
  t.ExecuteMotion('objs2')

  t.SetAttr(gripper,'x', t.robot.FK(arm=LEFT))
  x_w_1= t.GetAttr(gripper,'x')
  print 'Before:',x_w_1

  t.ExecuteMotion('rtposeest','clear',sensor_type)
  t.ExecuteMotion('rtposeest','create',sensor_type,'scene1',[gripper])

  t.ExecuteMotion('rtposeest','once',sensor_type,gripper)
  x_w_2= t.GetAttr(gripper,'x')
  print 'After:',x_w_2
  #t.ExecuteMotion('rtposeest','run',sensor_type,[gripper])

  #t.ExecuteMotion('rtposeest','remove',sensor_type,'scene1')

  print 'Current t.ar_adjust_ratio=',t.ar_adjust_ratio
  t.ar_adjust_ratio= 0.0

  x_sensor2= TransformRightInv(x_w_1,TransformLeftInv(t.x_sensor,x_w_2))
  t.x_sensor= x_sensor2

