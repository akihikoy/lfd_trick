#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move left arm/gripper to a posture for calibrating the marker on the wrist.
  Usage: initcmw'''
def Run(t,*args):
  angles= [0.30673466203417177, 0.3121264723011105, 1.591347118143797, -1.6316838948419994, 15.578274459980694, -1.4482715285671226, 39.269916223369705]
  t.robot.MoveToQ(angles,dt=3.0, arm=LEFT,blocking=False)

