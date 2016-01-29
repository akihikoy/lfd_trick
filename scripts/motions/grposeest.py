#!/usr/bin/python
from core_tool import *
def Help():
  return '''Active pose estimation using a m100 sensor on the left gripper.
  Usage: grposeest OBJS [, RTMETHOD [, MODE]]
    OBJS: list of object labels. e.g. ['b1'].
    RTMETHOD: Method of rtposeest (default: 'xyz').
    MODE: select from follows (default: 'rt').
      'rt': using ray tracing pose estimator.
      'test': testing motion only.
  '''
def Run(t,*args):
  l= TContainer(debug=True)
  l.objs= args[0]
  l.rtmethod= args[1] if len(args)>1 else 'xyz'
  l.mode= args[2] if len(args)>2 else 'rt'

  if not isinstance(l.objs,list):
    raise Exception('1st argument of grposeest should be a list')

  l.conservative= t.GetAttrOr(False, CURR,'conservative')

  #Setup collision check scene
  if not t.HasAttr(CURR,'scene'):
    t.SetAttr(CURR,'scene', l.objs)
  else:
    sobjs= t.GetAttr(CURR,'scene')
    for obj in l.objs:
      if obj not in sobjs:
        sobjs.append(obj)
    t.SetAttr(CURR,'scene', sobjs)


  def MoveToObservationPoint(obj):
    #t.GetAttr('wl_m100','lx') should be the same as lx_sensor_lg in rtposeest
    #NOTE:Following 3 lines worked with PR2.  The latter code should be general but has not tested yet.
    #invq= MultiplyQ(QFromAxisAngle([0,0,1],DegToRad(-90.)),QFromAxisAngle([0,1,0],DegToRad(-90.)))
    #lx_sensor_lg= copy.deepcopy(t.GetAttr('wl_m100','lx'))
    #lx_sensor_lg[3:]= Transform(invq, lx_sensor_lg[3:])
    lx_sensor_lg= copy.deepcopy(t.GetAttr('wl_m100','lx'))
    ex,ey,ez= RotToExyz(XToPosRot(lx_sensor_lg)[1])
    invq= MultiplyQ(QFromAxisAngle(ez,DegToRad(-90.)),QFromAxisAngle(ey,DegToRad(-90.)))
    lx_sensor_lg[3:]= Transform(invq, lx_sensor_lg[3:])

    #Plan a collision free observation pose around the object
    situation= {}
    situation['obj']= obj
    situation['handid']= LEFT
    situation['l_x_obs']= lx_sensor_lg
    situation['objs_occl']= set(l.objs)-set(obj)
    t.ExecuteMotion('infer2', situation,[],'obspose')
    x_slg_trg= t.GetAttrOr(None, obj,'x_obs_trg')
    if x_slg_trg==None:
      print 'Failed to infer x_obs_trg'
      return FailureCode('infer_x_obs_trg')

    exec_status= t.ExecuteMotion('move_to_x', x_slg_trg, 5.0, lx_sensor_lg, LEFT, {}, l.conservative)
    return exec_status

  if l.mode=='rt':
    t.ExecuteMotion('rtposeest','clear','lgrip')

  for obj in l.objs:
    l.exec_status= MoveToObservationPoint(obj)
    if not IsSuccess(l.exec_status):
      return l.exec_status
    if l.mode=='rt':
      ##Constrain the obj's pose to be upright on the table:
      #t.ExecuteMotion('infer2', {},[obj],'x')
      #x_o= t.GetAttr(obj,'x')
      #if t.HasAttr(obj,'base_marker_id'):
        #t.DelAttr(obj,'base_marker_id')
      #R= QToRot(x_o[3:])
      #e_z= R[:,2]
      #axis,angle= GetAxisAngle(e_z,[0.0,0.0,1.0])
      #x_o[3:]= MultiplyQ(QFromAxisAngle(axis,angle),x_o[3:])
      #t.SetAttr(obj,'x', x_o)

      t.ExecuteMotion('rtposeest', 'create', 'lgrip', 'scene'+obj,[obj])
      #t.ExecuteMotion('rtposeest', 'once', 'lgrip', obj, 10,'xyzrot2dxy')
      #t.ExecuteMotion('rtposeest', 'once', 'lgrip', obj, 10,'lin2dxy')
      t.ExecuteMotion('rtposeest', 'once', 'lgrip', obj, 10,l.rtmethod)

      #t.ExecuteMotion('rtposeest','remove','lgrip','scene'+obj)
      t.ExecuteMotion('rtposeest','clear','lgrip')

  return SUCCESS_CODE

