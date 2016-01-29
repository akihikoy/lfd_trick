#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move arm back to the position where it grabbed the object.
  Usage: backtog OBJ_ID [, HAND]
    OBJ_ID: identifier of object. e.g. 'b1'
    HAND: 'l': left hand, 'r': right hand (default: 'l')'''
def Run(t,*args):
  obj= args[0] if len(args)>0 else t.GetAttr(CURR,'source')

  if not t.HasAttr(obj,'grabbed'):
    print 'Error: not grabbed: ',obj
    return

  handid= t.GetAttr(obj,'grabbed','grabber_handid')

  target= t.GetAttrOr(None,obj,'grabbed','joint_angles')
  if target!=None:
    print 'Move back to q=',target
    t.robot.MoveToQ(target, 4.0, arm=handid, blocking=False)
  else:
    print 'Error: joint_angles is not assigned: ',obj

