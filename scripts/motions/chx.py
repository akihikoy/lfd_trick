#!/usr/bin/python
from core_tool import *
def Help():
  return '''Change pose (x) of object.
  Usage: chx OBJ [, METHOD [, EST_FIRST]]
    OBJ: label of an object (e.g. 'b1').
    METHOD: one of follows (default: 'upright').
      'upright': change the object orientation to be upright ([0,0,1] direction).
      'ontable': change the object height to be on the table.
    EST_FIRST: estimate the object pose before execution (default: True).
  '''
def Run(t,*args):
  obj= args[0]
  method= args[1] if len(args)>1 else 'upright'
  est_first= args[2] if len(args)>2 else True

  if est_first:
    t.ExecuteMotion('infer2', {},[obj],'x')
  x_o= t.GetAttr(obj,'x')

  if t.HasAttr(obj,'base_marker_id'):
    t.DelAttr(obj,'base_marker_id')

  if method=='upright':
    #change the object orientation to be upright ([0,0,1] direction):
    R= QToRot(x_o[3:])
    e_z= R[:,2]
    axis,angle= GetAxisAngle(e_z,[0.0,0.0,1.0])
    x_o[3:]= MultiplyQ(QFromAxisAngle(axis,angle),x_o[3:])
    t.SetAttr(obj,'x', x_o)

  elif method=='ontable':
    x_o[2]= t.GetAttr('table','x')[2]
    t.SetAttr(obj,'x', x_o)

  return x_o

