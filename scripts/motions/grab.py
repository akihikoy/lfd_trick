#!/usr/bin/python
from core_tool import *
def Help():
  return '''Grab an object.
  Usage: grab OBJ_ID [, HAND]
    OBJ_ID: identifier of object. e.g. 'b1'
    HAND: 'l': left hand, 'r': right hand (default: 'l')'''
def Run(t,*args):
  obj= args[0] if len(args)>0 else t.GetAttr(CURR,'source')
  hand= 'l'
  if len(args)>=2:  hand= args[1]
  handid= StrToLR(hand)

  if t.HasAttr(obj,'grabbed'):
    print 'Error: already grabbed: ',obj
    return FAILURE_PRECOND

  print 'Grabbing',obj,', which is ',t.GetAttrOr('(No description)',obj,'help')

  g_width= t.GetAttrOr(None,obj,'g_width')
  if g_width==None:
    g_width= 0.0
    print 'This object',obj,'does not have g_width attribute'
    print 'Use',g_width
    print 'OK?'
    if not t.AskYesNo():
      return FAILURE_PRECOND

  #Disabling ar sensor calibration using the marker on the gripper
  #to avoid undesirable calibration.
  #This will be enabled in release.
  t.ar_adjust_ratio= 0.0

  if t.robot.Is('PR2'):
    t.robot.MoveGripper(g_width, t.GetAttr(obj,'f_grab'), arm=handid, blocking=True)
  elif t.robot.Is('Baxter'):
    CPrint(0,'WARNING: attribute f_grab might be adjusted for PR2.',t.GetAttr(obj,'f_grab'))
    t.robot.MoveGripper(g_width, t.GetAttr(obj,'f_grab'), arm=handid, blocking=True)
  grab_attr= {}
  grab_attr['grabber']= 'gripper_'+hand
  grab_attr['grabber_wrist']= 'wrist_'+hand
  grab_attr['grabber_hand']= hand
  grab_attr['grabber_handid']= handid
  grab_attr['joint_angles']= t.robot.Q(handid)
  t.AddDictAttr(obj,'grabbed',  grab_attr)

  t.ExecuteMotion('infer', obj,'x')  #Should be inferred from grab info
  x_o= t.GetAttr(obj,'x')
  t.SetAttr(obj,'grabbed','grabbed_x', x_o)

  #Once grabbing, we do not use inference from the base marker
  t.DelAttr(obj,'base_marker_id')

  return SUCCESS_CODE
