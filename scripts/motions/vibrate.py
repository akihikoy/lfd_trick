#!/usr/bin/python
from core_tool import *
import copy
def Help():
  return '''Script to vibrate.
  Usage: vibrate'''
def Run(t,*args):
  #Use right hand
  handid= RIGHT

  #Local vector to the current control frame
  if handid==RIGHT:
    l_cf_e= t.GetAttr('wrist_r','lx')
  elif handid==LEFT:
    l_cf_e= t.GetAttr('wrist_l','lx')

  xe= t.robot.FK(x_ext=l_cf_e, arm=handid)
  dt= 0.3

  x_trg= copy.deepcopy(xe)
  x_trg[2]-= 0.015

  for i in range(7):
    t.robot.MoveToXI(x_trg,dt/2.0,l_cf_e,inum=5, arm=handid,blocking='time')
    t.robot.MoveToXI(xe,dt/2.0,l_cf_e,inum=5, arm=handid,blocking='time')
