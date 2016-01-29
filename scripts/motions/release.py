#!/usr/bin/python
from core_tool import *
def Help():
  return '''Release an object from gripper.
  Usage: release OBJ_ID
    OBJ_ID: identifier of object. e.g. 'b1' '''
def Run(t,*args):
  obj= args[0] if len(args)>0 else t.GetAttr(CURR,'source')

  if not t.HasAttr(obj,'grabbed'):
    print 'Error: not grabbed: ',obj
    return FAILURE_PRECOND

  handid= t.GetAttr(obj,'grabbed','grabber_handid')
  t.DelAttr(obj,'grabbed')

  if t.robot.Is('PR2'):
    t.robot.MoveGripper(0.09, 50, arm=handid, blocking=True)
  elif t.robot.Is('Baxter'):
    t.robot.MoveGripper(0.09, 50, arm=handid, blocking=True)

  #Enabling ar sensor calibration using the marker on the gripper.
  t.ar_adjust_ratio= t.default_ar_adjust_ratio

  return SUCCESS_CODE

