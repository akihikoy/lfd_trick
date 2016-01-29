#!/usr/bin/python
from core_tool import *
import time
def Help():
  return '''Register a base point (bp) by moving gripper.
  Usage: setbp BP_ID'''
def Run(t,*args):
  bpid= args[0]
  arm= t.robot.Arm
  t.robot.CloseGripper(arm=arm, blocking=True)
  print '###CAUTION:',t.robot.ArmStr,'arm is relaxed'
  t.robot.Mannequin.Activate(arm)
  print 'Move',t.robot.ArmStr,'arm to a point that you want to register'
  print 'Is it OK?'
  if t.AskYesNo():
    print '###CAUTION:',t.robot.ArmStr,'arm is fixed'
    t.robot.Mannequin.Deactivate(arm)
    time.sleep(0.5)  # Wait that the FK observation is updated
    if arm==RIGHT:
      xe= t.robot.FK(x_ext=t.GetAttr('wrist_r','lx'), arm=arm)
    elif arm==LEFT:
      xe= t.robot.FK(x_ext=t.GetAttr('wrist_l','lx'), arm=arm)
    print 'xe= ',VecToStr(xe)
    print 'Do you want to change the orientation to [0,0,0,1]?'
    if t.AskYesNo():
      xe[3:7]= [0.0,0.0,0.0,1.0]
    t.base_x[bpid]= xe
    print 'bp add',bpid,VecToStr(t.base_x[bpid])
    print 'Done'
  else:
    print '###CAUTION:',t.robot.ArmStr,'arm is fixed'
    t.robot.Mannequin.Deactivate(arm)
    print 'Canceled'
