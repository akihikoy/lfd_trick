#!/usr/bin/python
from core_tool import *
def Help():
  return '''Pre-pouring motion where the bottle should be grabbed.
  Usage: prepour BOTTLE_ID, CUP_ID
    BOTTLE_ID: identifier of bottle. e.g. 'b1'
    CUP_ID: identifier of cup. e.g. 'c1' '''
def Run(t,*args):
  bottle= args[0] if len(args)>0 else t.GetAttr(CURR,'source')
  cup= args[1] if len(args)>1 else t.GetAttr(CURR,'receiver')

  if not t.HasAttr(bottle,'grabbed'):
    print 'Error: not grabbed: ',bottle
    return FAILURE_PRECOND

  m_ctrl= t.LoadMotion('ctrl')

  l= TContainer(debug=True)
  l.bottle= bottle
  l.cup= cup
  l.handid= t.GetAttr(l.bottle,'grabbed','grabber_handid')
  l.conservative= t.GetAttrOr(False, CURR,'conservative')

  #Pouring edge point in the wrist frame (control point)
  lb_x_pour_e= t.GetAttr(l.bottle,'l_x_pour_e')
  t.ExecuteMotion('infer', l.bottle,'x')
  x_b= t.GetAttr(l.bottle,'x')
  x_pour_e= Transform(x_b,lb_x_pour_e)
  x_w= t.robot.FK(arm=l.handid)
  l.lw_x_pour_e= TransformLeftInv(x_w, x_pour_e)
  print 'lw_x_pour_e=',VecToStr(l.lw_x_pour_e)


  #Pouring location on the cup frame:
  def GetXPourL():
    t.ExecuteMotion('infer', l.cup,'x')
    x_c= t.GetAttr(l.cup,'x')
    return Transform(x_c, t.GetAttr(l.cup,'l_x_pour_l'))

  def ChangeXUpward(x_curr):
    x_res= copy.deepcopy(x_curr)
    x_res[2]+= 0.10  #FIXME: magic parameter
    return x_res

  l.move_upward= m_ctrl.TMoveDiffX()
  l.move_upward.core_tool= t
  l.move_upward.change_x= ChangeXUpward
  l.move_upward.l_x_ext= l.lw_x_pour_e
  l.move_upward.arm= l.handid
  #Move up until collision is free or 10cm
  l.move_upward.init_callback= lambda:t.ExecuteMotion('scene', 'make',[],1.2)  #1.2 is bigger margin than that is used in infer_traj (called from move_to_x)
  l.move_upward.additional_check= lambda:not (t.ExecuteMotion('scene','isvalidq',l.handid,[],t.robot.Q(l.handid)))[0]
  l.move_upward.exit_callback= lambda:t.ExecuteMotion('scene', 'clear')


  def MoveToPourL0():
    t.ExecuteMotion('infer', l.cup,'x')
    x_c= t.GetAttr(l.cup,'x')
    x_pour_l0= Transform(x_c, t.GetAttr(l.cup,'l_x_pour_l0'))

    #Future use (for moving back):
    t.SetAttr(CURR,'lw_x_pour_e', l.lw_x_pour_e)
    t.SetAttr(CURR,'x_pour_l0', x_pour_l0)

    #Move pouring edge to pouring location (0)
    #t.robot.MoveToX(x_pour_l0,3.0,l.lw_x_pour_e,True, arm=l.handid)
    l.exec_status= t.ExecuteMotion('move_to_x', x_pour_l0, 5.0, l.lw_x_pour_e, l.handid, {}, l.conservative)

  l.shift_to_pour_l= m_ctrl.TApproachToX()
  l.shift_to_pour_l.core_tool= t
  l.shift_to_pour_l.get_x_trg= GetXPourL
  l.shift_to_pour_l.l_x_ext= l.lw_x_pour_e
  l.shift_to_pour_l.arm= l.handid
  #Show collisions (with a strict margin 1.0)
  l.shift_to_pour_l.init_callback= lambda:t.ExecuteMotion('scene', 'make',[],1.0)
  #l.shift_to_pour_l.additional_check= lambda:(
      #t.ExecuteMotion('scene','isvalidq',l.handid,[],t.robot.Q(l.handid),True),
      #True)[-1]
  #Stop at collision
  l.shift_to_pour_l.additional_check= lambda:(t.ExecuteMotion('scene','isvalidq',l.handid,[],t.robot.Q(l.handid),True))[0]
  l.shift_to_pour_l.exit_callback= lambda:t.ExecuteMotion('scene', 'clear')

  #Run pose estimators to adjust the displacement of containers at pouring location (0)
  def CalibAtPourL0():
    if t.GetAttr(CURR,'rtposeest_mode')=='xtion':
      #Calibrate the RGB-D sensor pose with the ray tracing pose estimation of
      #grabbed container:
      t.ExecuteMotion('rtcalib_x', l.bottle)
      #Run the ray tracing pose estimator for the receiver:
      t.ExecuteMotion('rtposeest', 'once', 'ext', l.cup, 3,'lin2dxy')


  sm= TStateMachine()
  sm.EventCallback= t.SMCallback
  sm.Debug= True

  sm.StartState= 'start'
  sm.NewState('start')
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'move_upward'

  sm.NewState('move_upward')
  sm['move_upward'].EntryAction= l.move_upward.Init
  sm['move_upward'].NewAction()
  sm['move_upward'].Actions[-1].Condition= l.move_upward.Check
  sm['move_upward'].Actions[-1].Action= l.move_upward.Step
  sm['move_upward'].Actions[-1].NextState= ORIGIN_STATE
  sm['move_upward'].ElseAction.Condition= lambda: True
  sm['move_upward'].ElseAction.Action= l.move_upward.Exit
  sm['move_upward'].ElseAction.NextState= 'move_to_pour_l0'

  sm.NewState('move_to_pour_l0')
  sm['move_to_pour_l0'].EntryAction= MoveToPourL0
  sm['move_to_pour_l0'].NewAction()
  sm['move_to_pour_l0'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['move_to_pour_l0'].Actions[-1].NextState= 'shift_to_pour_l'
  sm['move_to_pour_l0'].ElseAction.Condition= lambda: True
  sm['move_to_pour_l0'].ElseAction.Action= sm.SetFailure
  sm['move_to_pour_l0'].ElseAction.NextState= EXIT_STATE

  sm.NewState('shift_to_pour_l')
  #sm['shift_to_pour_l'].EntryAction= l.shift_to_pour_l.Init
  sm['shift_to_pour_l'].EntryAction= lambda:( CalibAtPourL0(), l.shift_to_pour_l.Init() )
  sm['shift_to_pour_l'].NewAction()
  sm['shift_to_pour_l'].Actions[-1].Condition= l.shift_to_pour_l.Check
  sm['shift_to_pour_l'].Actions[-1].Action= l.shift_to_pour_l.Step
  sm['shift_to_pour_l'].Actions[-1].NextState= ORIGIN_STATE
  sm['shift_to_pour_l'].NewAction()
  sm['shift_to_pour_l'].Actions[-1].Condition= lambda: IsFailure(l.shift_to_pour_l.status)
  sm['shift_to_pour_l'].Actions[-1].Action= lambda: ( sm.SetFailure(), l.shift_to_pour_l.Exit() )
  sm['shift_to_pour_l'].Actions[-1].NextState= EXIT_STATE
  sm['shift_to_pour_l'].ElseAction.Condition= lambda: True
  sm['shift_to_pour_l'].ElseAction.Action= l.shift_to_pour_l.Exit
  sm['shift_to_pour_l'].ElseAction.NextState= EXIT_STATE


  t.RunSM(sm,'prepour')
  l= None


  return sm.ExitStatus

