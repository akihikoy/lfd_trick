#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move left arm/gripper to calibration posture.
  Usage: initc'''
def Run(t,*args):
  #angles= [0.8, 0.0, 1.57, -1.5, 3.14, 0,0]
  #angles= [0.2943815760479809, 0.4523002598851919, 1.7590782665266982, -1.5979523190896374, 16.15402651639962, -0.9139384808240009, 0.0]
  angles= [0.29786365397697423, 0.38394756201679253, 1.6869185946602492, -1.6458714245575852, 9.556739790949049, -0.9139819897441014, 0.0]
  #t.robot.MoveGripper(0.09, 50, arm=LEFT, blocking=False)
  t.robot.MoveToQ(angles,dt=3.0, arm=LEFT,blocking=False)

  t.robot.MoveGripper(0.005, 50, arm=LEFT, blocking=False)
  print 'Insert the calibration stick. Ready?'
  if t.AskYesNo():
    t.robot.MoveGripper(0.0, 50, arm=LEFT, blocking=False)

