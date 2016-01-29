#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move left arm/gripper to init Cartesian position.
  Usage: initx0'''
def Run(t,*args):
  conservative= True
  if t.robot.Is('PR2'):
    x0= [0.35995997222152437, 0.0799988455779772, -0.17998988886047518, -0.01755478120593006, 0.21304820655770998, 0.08625015236489832, 0.97306896084316]
  elif t.robot.Is('Baxter'):
    x0= [0.5846745479601083, 0.232635085553269, 0.1471978844978266, 0.18756196167585984, 0.7881231847633849, -0.048399466992405285, 0.5842429698041497]
  t.robot.OpenGripper(arm=LEFT, blocking=False)
  exec_status= t.ExecuteMotion('move_to_x', x0, 4.0, [0.,0.,0., 0.,0.,0.,1.], LEFT, {}, conservative)
