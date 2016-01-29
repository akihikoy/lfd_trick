#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move left arm/gripper to init posture.
  Usage: init0'''
def Run(t,*args):
  if t.robot.Is('PR2'):
    angles= [-1.5758421026969418, 1.2968352230407523, -1.6520923310211921, -2.095963566248973, 10.512690320637843, -1.469029183486648, 2.37512293699]
  elif t.robot.Is('Baxter'):
    angles= [0.6772525170776368, -0.8617137066101075, -0.1092961310119629, 2.4812139215698243, -0.7577865083496095, -1.4657186411499024, -0.12732040524902344]
    angles[0]-= 0.6
  t.robot.OpenGripper(arm=RIGHT, blocking=False)
  t.robot.MoveToQ(angles,dt=4.0, arm=RIGHT,blocking=False)
