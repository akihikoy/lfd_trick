#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move endeffector to initial pose where the task started.
  Usage: init1 [, DURATION]
    DURATION: Duration to get to the pose in seconds (default: 3.0).
  '''
def Run(t,*args):
  dt= args[0] if len(args)>0 else 3.0

  assert(t.HasAttr(CURR,'init_x'))
  assert(t.HasAttr(CURR,'init_handid'))

  x_init= t.GetAttr(CURR,'init_x')
  handid= t.GetAttr(CURR,'init_handid')
  lw_xe= t.GetAttr('wrist_'+LRToStrs(handid),'lx')  #TODO: this should be stored
  conservative= t.GetAttrOr(False, CURR,'conservative')
  return t.ExecuteMotion('move_to_x', x_init, dt, lw_xe, handid, {}, conservative)
