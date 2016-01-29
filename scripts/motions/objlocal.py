#!/usr/bin/python
from core_tool import *
def Help():
  return '''Template of script.
  Usage: objlocal OBJ_ID, POSE
    OBJ_ID: identifier of object. e.g. 'b1'
    POSE: position and quaternion in the robot frame'''
def Run(t,*args):
  obj= args[0]
  x= args[1]

  #Infer the object's pose in robot frame
  t.ExecuteMotion('infer', obj,'x')
  x_o= t.GetAttr(obj,'x')

  l_x= TransformLeftInv(x_o, x)

  print 'Pose on',obj,'frame: ',l_x
