#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move to an object to grab.
  Usage: pregrab OBJ_ID [, HAND]
    OBJ_ID: identifier of object. e.g. 'b1'
    HAND: 'l': left hand, 'r': right hand (default: 'l')'''
def Run(t,*args):
  obj= args[0] if len(args)>0 else t.GetAttr(CURR,'source')
  hand= args[1] if len(args)>1 else 'l'

  m_ctrl= t.LoadMotion('ctrl')

  l= TContainer(debug=True)
  l.obj= obj
  l.handid= StrToLR(hand)
  l.lw_xe= t.GetAttr('wrist_'+hand,'lx')
  l.conservative= t.GetAttrOr(False, CURR,'conservative')

  if not t.HasAttr(CURR,'init_x'):
    t.SetAttr(CURR,'init_x', t.robot.FK(x_ext=l.lw_xe,arm=l.handid))
    t.SetAttr(CURR,'init_handid', l.handid)

  def GetXGrab0():
    t.ExecuteMotion('infer', l.obj,'x')
    x_o= t.GetAttr(l.obj,'x')
    lo_x_grab0= t.GetAttr(l.obj,'l_x_grab0')
    x_grab0= Transform(x_o,lo_x_grab0)
    return x_grab0

  def GetXGrab():
    t.ExecuteMotion('infer', l.obj,'x')
    x_o= t.GetAttr(l.obj,'x')
    lo_x_grab= t.GetAttr(l.obj,'l_x_grab')
    x_grab= Transform(x_o,lo_x_grab)
    return x_grab

  def OpenGrip():
    #Open the gripper to 'g_pre' value
    if t.HasAttr(l.obj,'g_pre'):
      if t.robot.Is('PR2'):
        t.robot.MoveGripper(t.GetAttr(l.obj,'g_pre'), 50, arm=l.handid, blocking=True)
      elif t.robot.Is('Baxter'):
        t.robot.MoveGripper(t.GetAttr(l.obj,'g_pre'), 50, arm=l.handid, blocking=True)
      return True
    return False

  #Move the gripper to front of the object
  def MoveToGrab0():
    t.ExecuteMotion('infer', l.obj,'x')
    x_o= t.GetAttr(l.obj,'x')
    lo_x_grab0= t.GetAttr(l.obj,'l_x_grab0')
    x_grab0= Transform(x_o,lo_x_grab0)

    #t.robot.MoveToX(x_grab0,3.0,l.lw_xe,True, arm=l.handid)
    l.exec_status= t.ExecuteMotion('move_to_x', x_grab0, 3.0, l.lw_xe, l.handid, {}, l.conservative)

  #Adjusting sensor calibration by the marker on the wrist
  def ARAdjust_Init():
    l.ar_adjust_ratio= t.ar_adjust_ratio
    t.ar_adjust_ratio= 0.2
    time.sleep(0.5)
    l.wait_counter=0
  #Return if we need to wait the ar adjustment
  def ARAdjust_Check():
    return t.ar_adjust_err>0.005 and l.wait_counter<20
  def ARAdjust_Step():
    print 'Waiting AR marker adjustment..',t.ar_adjust_err
    time.sleep(0.2)
    l.wait_counter+=1
  def ARAdjust_Exit():
    t.ar_adjust_ratio= l.ar_adjust_ratio

  #Estimating bottle pose again with ray tracing estimator
  def RTPoseAdjust():
    if t.GetAttr(CURR,'rtposeest_mode')=='cmb':
      t.ExecuteMotion('rtposeest', 'create', 'lgrip', 'scene'+l.obj,[l.obj])
      #t.ExecuteMotion('rtposeest', 'once', 'lgrip', l.obj, 5,'xyz')
      t.ExecuteMotion('rtposeest', 'once', 'lgrip', l.obj, 5,'lin2dxy')
    l.exec_status= SUCCESS_CODE


  l.shift_to_grab0= m_ctrl.TApproachToX()
  l.shift_to_grab0.core_tool= t
  l.shift_to_grab0.get_x_trg= GetXGrab0
  l.shift_to_grab0.l_x_ext= l.lw_xe
  l.shift_to_grab0.arm= l.handid

  l.shift_to_grab= m_ctrl.TApproachToX()
  l.shift_to_grab.core_tool= t
  l.shift_to_grab.get_x_trg= GetXGrab
  l.shift_to_grab.l_x_ext= l.lw_xe
  l.shift_to_grab.arm= l.handid
  #Show collisions
  l.shift_to_grab.init_callback= lambda:t.ExecuteMotion('scene', 'make')
  l.shift_to_grab.additional_check= lambda:(
      t.ExecuteMotion('scene','isvalidq',l.handid,[],t.robot.Q(l.handid),True),  #Just show the collision points
      True)[-1]
  #Stop at collision
  #l.shift_to_grab.additional_check= lambda:(t.ExecuteMotion('scene','isvalidq',l.handid,[],t.robot.Q(l.handid),True))[0]
  l.shift_to_grab.exit_callback= lambda:t.ExecuteMotion('scene', 'clear')



  sm= TStateMachine()
  sm.EventCallback= t.SMCallback
  sm.Debug= True

  sm.StartState= 'start'
  sm.NewState('start')
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'open_grip'

  sm.NewState('open_grip')
  sm['open_grip'].NewAction()
  sm['open_grip'].Actions[-1].Condition= OpenGrip
  sm['open_grip'].Actions[-1].NextState= 'move_to_grab0'
  sm['open_grip'].ElseAction.Condition= lambda: True
  sm['open_grip'].ElseAction.Action= sm.SetFailure
  sm['open_grip'].ElseAction.NextState= EXIT_STATE

  sm.NewState('move_to_grab0')
  sm['move_to_grab0'].EntryAction= MoveToGrab0
  sm['move_to_grab0'].NewAction()
  sm['move_to_grab0'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['move_to_grab0'].Actions[-1].NextState= 'rt_pose_adjust' if t.GetAttr(CURR,'rtposeest_mode')=='cmb' else 'wait_ar_adjust'
  sm['move_to_grab0'].ElseAction.Condition= lambda: True
  sm['move_to_grab0'].ElseAction.Action= sm.SetFailure
  sm['move_to_grab0'].ElseAction.NextState= EXIT_STATE

  sm.NewState('wait_ar_adjust')
  sm['wait_ar_adjust'].EntryAction= ARAdjust_Init
  sm['wait_ar_adjust'].NewAction()
  sm['wait_ar_adjust'].Actions[-1].Condition= ARAdjust_Check
  sm['wait_ar_adjust'].Actions[-1].Action= ARAdjust_Step
  sm['wait_ar_adjust'].Actions[-1].NextState= ORIGIN_STATE
  sm['wait_ar_adjust'].ElseAction.Condition= lambda: True
  sm['wait_ar_adjust'].ElseAction.Action= ARAdjust_Exit
  sm['wait_ar_adjust'].ElseAction.NextState= 'shift_to_grab0'

  ##RGB-D sensor calibration using pose estimation of gripper with ray tracing pose estimator.
  #sm.NewState('wait_ar_adjust')
  #sm['wait_ar_adjust'].EntryAction= lambda: t.ExecuteMotion('rtcalib_x','wrist_l') if t.GetAttr(CURR,'rtposeest_mode')=='xtion' else None
  #sm['wait_ar_adjust'].ElseAction.Condition= lambda: True
  #sm['wait_ar_adjust'].ElseAction.NextState= 'shift_to_grab0'

  sm.NewState('rt_pose_adjust')
  sm['rt_pose_adjust'].EntryAction= RTPoseAdjust
  sm['rt_pose_adjust'].NewAction()
  sm['rt_pose_adjust'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['rt_pose_adjust'].Actions[-1].NextState= 'shift_to_grab0'
  sm['rt_pose_adjust'].ElseAction.Condition= lambda: True
  sm['rt_pose_adjust'].ElseAction.Action= sm.SetFailure
  sm['rt_pose_adjust'].ElseAction.NextState= EXIT_STATE

  sm.NewState('shift_to_grab0')
  sm['shift_to_grab0'].EntryAction= l.shift_to_grab0.Init
  sm['shift_to_grab0'].NewAction()
  sm['shift_to_grab0'].Actions[-1].Condition= l.shift_to_grab0.Check
  sm['shift_to_grab0'].Actions[-1].Action= l.shift_to_grab0.Step
  sm['shift_to_grab0'].Actions[-1].NextState= ORIGIN_STATE
  sm['shift_to_grab0'].NewAction()
  sm['shift_to_grab0'].Actions[-1].Condition= lambda: IsFailure(l.shift_to_grab0.status)
  sm['shift_to_grab0'].Actions[-1].Action= lambda: ( sm.SetFailure(), l.shift_to_grab0.Exit() )
  sm['shift_to_grab0'].Actions[-1].NextState= EXIT_STATE
  sm['shift_to_grab0'].ElseAction.Condition= lambda: True
  sm['shift_to_grab0'].ElseAction.Action= l.shift_to_grab0.Exit
  sm['shift_to_grab0'].ElseAction.NextState= 'shift_to_grab'

  sm.NewState('shift_to_grab')
  sm['shift_to_grab'].EntryAction= l.shift_to_grab.Init
  sm['shift_to_grab'].NewAction()
  sm['shift_to_grab'].Actions[-1].Condition= l.shift_to_grab.Check
  sm['shift_to_grab'].Actions[-1].Action= l.shift_to_grab.Step
  sm['shift_to_grab'].Actions[-1].NextState= ORIGIN_STATE
  sm['shift_to_grab'].NewAction()
  sm['shift_to_grab'].Actions[-1].Condition= lambda: IsFailure(l.shift_to_grab.status)
  sm['shift_to_grab'].Actions[-1].Action= lambda: ( sm.SetFailure(), l.shift_to_grab.Exit() )
  sm['shift_to_grab'].Actions[-1].NextState= EXIT_STATE
  sm['shift_to_grab'].ElseAction.Condition= lambda: True
  sm['shift_to_grab'].ElseAction.Action= l.shift_to_grab.Exit
  sm['shift_to_grab'].ElseAction.NextState= EXIT_STATE


  t.RunSM(sm,'pregrab')
  l= None

  return sm.ExitStatus
