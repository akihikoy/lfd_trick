#!/usr/bin/python
from core_tool import *
def Help():
  return '''Control test.
  Usage: test_ctrl'''
def Run(t,*args):
  m_ctrl= t.LoadMotion('ctrl')
  arm= LEFT
  obj= 'b53'
  lw_xe= t.GetAttr('wrist_'+LRToStrs(arm),'lx')
  ctrl_test_only= True

  if not ctrl_test_only:
    #Rough estimation of an object
    xe0= t.robot.FK(arm=arm,x_ext=lw_xe)
    ex,ey,ez= RotToExyz(QToRot(xe0[3:]))
    ex[2]= 0.0
    xe0[:3]= xe0[:3]+0.30*ex
    xe0[2]-= 0.10
    xe0[3:]= [0.,0.,0.,1.]
    t.SetAttr(obj,'x', xe0)
    t.SetAttr(obj+'dummy', copy.deepcopy(t.GetAttr(obj)))

    t.ExecuteMotion('viz', obj, obj+'dummy')

    #Target object pose estimation with M100 on gripper
    t.ExecuteMotion('rtposeest', 'create', 'lgrip', 'scene'+obj,[obj])
    t.ExecuteMotion('rtposeest', 'once', 'lgrip', obj, 5,'xyz')
    #t.ExecuteMotion('rtposeest', 'once', 'lgrip', obj, 5,'lin2dxy')

    x_o= t.GetAttr(obj,'x')
    x_grab0= t.robot.FK(arm=arm,x_ext=lw_xe)
    ex,ey,ez= RotToExyz(QToRot(x_grab0[3:]))
    x_grab0[:2]= (x_o[:3]-0.30*ex)[:2]
  else:
    x_grab0= t.robot.FK(arm=arm,x_ext=lw_xe)
    x_grab0[0]+= 0.01
    t.ExecuteMotion('viz', '')

  t.viz.test= TSimpleVisualizer(name_space='visualizer_test')
  t.viz.test.viz_frame= t.robot.BaseFrame
  t.viz.test.AddMarker(x_grab0, scale=[0.05,0.04,0.012], rgb=t.viz.test.ICol(2), alpha=0.5)

  #Approaching control
  shift_to_grab0= m_ctrl.TApproachToX()
  shift_to_grab0.core_tool= t
  shift_to_grab0.get_x_trg= lambda:x_grab0
  shift_to_grab0.l_x_ext= lw_xe
  shift_to_grab0.arm= arm

  shift_to_grab0.Init()
  while shift_to_grab0.Check():
    shift_to_grab0.Step()
  ##Activate PI control:  (NOTE: integrated into m_ctrl.TApproachToX)
  #dt= 0.01
  #xd_sum= Vec([0.0]*6)
  #while t.AskYesNo():
    #for i in range(50):
      #x_curr= t.robot.FK(arm=arm,x_ext=lw_xe)
      #x_diff= Vec(DiffX(x_curr, x_grab0))
      #xd_sum+= x_diff*dt
      #print Norm(x_diff[:3]),Norm(x_diff[3:])
      #t.robot.MoveToX(AddDiffX(x_grab0,0.5*xd_sum),dt,lw_xe,arm=arm,blocking='time')
  shift_to_grab0.Exit()

  raw_input('done-1 > ')
  #print 'Target=',x_grab0
  #print 'Current=',t.robot.FK(arm=arm,x_ext=lw_xe)
  print 'Position error=',(Vec(x_grab0[:3])-t.robot.FK(arm=arm,x_ext=lw_xe)[:3])*1000.0
  #There is 5 mm error!

  #This does not reduce the error:
  t.robot.MoveToX(x_grab0,1.0,lw_xe,arm=arm,blocking=True)
  raw_input('done-2 > ')
  print 'Position error=',(Vec(x_grab0[:3])-t.robot.FK(arm=arm,x_ext=lw_xe)[:3])*1000.0

  ##This does not reduce the error (i.e. NOT IK problem):
  #q_grab0= t.robot.IK(x_grab0, x_ext=lw_xe, arm=arm)
  #if q_grab0 is not None:  t.robot.MoveToQ(q_grab0,1.0, arm=arm,blocking=True)
  #else:  CPrint(4,'No IK solution found')
  #raw_input('done-3 > ')
  #print 'Position error=',(Vec(x_grab0[:3])-t.robot.FK(arm=arm,x_ext=lw_xe)[:3])*1000.0
  #print 'IK error=',(Vec(x_grab0[:3])-t.robot.FK(q=q_grab0,arm=arm,x_ext=lw_xe)[:3])*1000.0
