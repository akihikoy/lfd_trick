#!/usr/bin/python
from core_tool import *
def Help():
  return '''Change the grasping pose to be suit for pouring.
  Usage: regrab OBJ_ID
    OBJ_ID: identifier of grasping object. e.g. 'b1'
  '''
def Run(t,*args):
  l= TContainer(debug=True)

  l.obj= args[0] if len(args)>0 else t.GetAttr(CURR,'source')


  #Check the initial condition:
  if not t.HasAttr(l.obj,'grabbed'):
    print 'Error: not grabbed: ',l.obj
    return FAILURE_PRECOND

  l.handid= t.GetAttr(l.obj,'grabbed','grabber_handid')
  l.lw_xe= t.GetAttr('wrist_'+LRToStrs(l.handid),'lx')
  l.conservative= t.GetAttrOr(False, CURR,'conservative')

  m_ctrl= t.LoadMotion('ctrl')


  def AssessReGrab(a, event_type, context, sm):
    MemorizeAsBad= lambda assess_key, assess_info, ask=True: \
        t.MemorizeAssessment(a, mem_key=['regrab','bad'], assess_key=assess_key, assess_info=assess_info, context=copy.deepcopy(context), ask=ask)

    exception_req= False
    if t.GetAttrOr(False, CURR,'human_assessed_bad') and not a.MarkedBad:
      CPrint(3, 'Was the inference "regrab" wrong?')
      if t.AskYesNo():
        if MemorizeAsBad(assess_key='human_assess', assess_info={'message':'bad'}, ask=False):
          exception_req= True

    if not IsSuccess(sm.ExitStatus):
      MemorizeAsBad(assess_key='auto', assess_info={'kind':sm.ExitStatus})
    if event_type==a.EVENT_EXIT and context==a.RegisteredContext[:-1]:
      a.Conditions= set()

    if t.GetAttrOr(False, CURR,'human_assessed_bad') and not exception_req:
      CPrint(3, 'Stop the motion now? (EXCEPTION)')
      if t.AskYesNo():  exception_req= True

    if exception_req:  return a.EXCEPTION
    return a.COMPLETED if len(a.Conditions)==0 else a.CONTINUE

  def InferReGrab():
    situation= {
      'obj':      l.obj,
      'handid':   l.handid,
      'l_x_ext':  l.lw_xe}
    inferred_keys_tmp= []
    if not t.ExecuteMotion('infer2', situation,[],'regrab',inferred_keys_tmp):
      l.exec_status= FailureCode('InferReGrab')
      return
    inferred_keys= t.GetAttrOr([], CURR,'inferred_keys') + inferred_keys_tmp
    t.SetAttr(CURR,'inferred_keys', inferred_keys)

    assess_regrab= TAssessment(TAssessment.STATE_CALLBACK, name='assess_regrab')
    assess_regrab.Assess= AssessReGrab
    assess_regrab.Conditions= set(['sm'])
    assess_regrab.DBIndex= t.GetAttr(CURR,'new_in_database')[-1]
    assess_regrab.MarkedBad= False
    t.RegisterAssessment(assess_regrab)
    l.exec_status= SUCCESS_CODE


  def ChangeXUpward(x_curr):
    x_res= copy.deepcopy(x_curr)
    x_res[2]+= 0.10  #FIXME: magic parameter
    return x_res

  l.move_upward= m_ctrl.TMoveDiffX()
  l.move_upward.core_tool= t
  l.move_upward.change_x= ChangeXUpward
  l.move_upward.l_x_ext= l.lw_xe
  l.move_upward.arm= l.handid
  #Move up until collision is free or 10cm
  l.move_upward.init_callback= lambda:t.ExecuteMotion('scene', 'make')
  l.move_upward.additional_check= lambda:not (t.ExecuteMotion('scene','isvalidq',l.handid,[],t.robot.Q(l.handid)))[0]
  l.move_upward.exit_callback= lambda:t.ExecuteMotion('scene', 'clear')


  def MoveToXPut0():
    x_put= copy.deepcopy(t.GetAttr(CURR,'regrab','x_put'))

    #Get a collision free pose above x_put
    x_put2= copy.deepcopy(x_put)
    t.ExecuteMotion('scene', 'make')
    for dz in FRange1(0.0, 0.10, 20):  #FIXME: magic parameter
      x_put2[2]= x_put[2]+dz
      if (t.ExecuteMotion('scene','isvalidx',l.handid,TransformRightInv(x_put2,l.lw_xe)))[0]:
        l.x_put0_dz= dz
        break
    t.ExecuteMotion('scene', 'clear')
    if 'x_put0_dz' not in l:
      CPrint(4,'Error: Cannot find x_put0_dz')
      l.exec_status= FailureCode('MoveToXPut0')
      return
    x_put[2]+= l.x_put0_dz

    l.exec_status= t.ExecuteMotion('move_to_x', x_put, 5.0, l.lw_xe, l.handid, {}, l.conservative)


  def ChangeXDownward(x_curr):
    x_res= copy.deepcopy(x_curr)
    x_res[2]-= l.x_put0_dz
    return x_res

  l.move_downward= m_ctrl.TMoveDiffX()
  l.move_downward.core_tool= t
  l.move_downward.change_x= ChangeXDownward
  l.move_downward.l_x_ext= l.lw_xe
  l.move_downward.arm= l.handid


  def Release():
    CPrint(3, 'Next motion: release')
    if l.conservative and not t.AskYesNo():
      l.exec_status= FailureCode('canceled')
    else:
      l.exec_status= t.ExecuteMotion('release', l.obj)


  def WithdrawX(x_curr):
    p,R= XToPosRot(x_curr)
    g_width= t.GetAttr(l.obj,'g_width')
    p= p - 2.0*g_width*R[:,0]  #Withdraw to ex direction
    p[2]+= 0.04  #4cm above; hack to avoid collision of arm with table
    return PosRotToX(p,R)

  l.shift_to_grab1= m_ctrl.TMoveDiffX()
  l.shift_to_grab1.core_tool= t
  l.shift_to_grab1.change_x= WithdrawX
  l.shift_to_grab1.l_x_ext= l.lw_xe
  l.shift_to_grab1.arm= l.handid
  #Move until collision is free or reaching WithdrawX
  l.shift_to_grab1.init_callback= lambda:t.ExecuteMotion('scene', 'make')
  l.shift_to_grab1.additional_check= lambda:not (t.ExecuteMotion('scene','isvalidq',l.handid))[0]
  l.shift_to_grab1.exit_callback= lambda:t.ExecuteMotion('scene', 'clear')



  sm= TStateMachine()
  sm.EventCallback= t.SMCallback
  sm.Debug= True

  sm.StartState= 'infer_regrab'

  sm.NewState('infer_regrab')
  sm['infer_regrab'].EntryAction= InferReGrab
  sm['infer_regrab'].NewAction()
  sm['infer_regrab'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['infer_regrab'].Actions[-1].NextState= 'move_upward'
  sm['infer_regrab'].ElseAction.Condition= lambda: True
  sm['infer_regrab'].ElseAction.Action= sm.SetFailure
  sm['infer_regrab'].ElseAction.NextState= EXIT_STATE

  sm.NewState('move_upward')
  sm['move_upward'].EntryAction= l.move_upward.Init
  sm['move_upward'].NewAction()
  sm['move_upward'].Actions[-1].Condition= l.move_upward.Check
  sm['move_upward'].Actions[-1].Action= l.move_upward.Step
  sm['move_upward'].Actions[-1].NextState= ORIGIN_STATE
  sm['move_upward'].ElseAction.Condition= lambda: True
  sm['move_upward'].ElseAction.Action= l.move_upward.Exit
  sm['move_upward'].ElseAction.NextState= 'move_to_x_put0'

  sm.NewState('move_to_x_put0')
  sm['move_to_x_put0'].EntryAction= MoveToXPut0
  sm['move_to_x_put0'].NewAction()
  sm['move_to_x_put0'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['move_to_x_put0'].Actions[-1].NextState= 'move_downward'
  sm['move_to_x_put0'].ElseAction.Condition= lambda: True
  sm['move_to_x_put0'].ElseAction.Action= sm.SetFailure
  sm['move_to_x_put0'].ElseAction.NextState= EXIT_STATE

  sm.NewState('move_downward')
  sm['move_downward'].EntryAction= l.move_downward.Init
  sm['move_downward'].NewAction()
  sm['move_downward'].Actions[-1].Condition= l.move_downward.Check
  sm['move_downward'].Actions[-1].Action= l.move_downward.Step
  sm['move_downward'].Actions[-1].NextState= ORIGIN_STATE
  sm['move_downward'].ElseAction.Condition= lambda: True
  sm['move_downward'].ElseAction.Action= l.move_downward.Exit
  sm['move_downward'].ElseAction.NextState= 'release'

  sm.NewState('release')
  sm['release'].EntryAction= Release
  sm['release'].NewAction()
  sm['release'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['release'].Actions[-1].NextState= 'shift_to_grab1'
  sm['release'].ElseAction.Condition= lambda: True
  sm['release'].ElseAction.Action= sm.SetFailure
  sm['release'].ElseAction.NextState= EXIT_STATE

  sm.NewState('shift_to_grab1')
  sm['shift_to_grab1'].EntryAction= l.shift_to_grab1.Init
  sm['shift_to_grab1'].NewAction()
  sm['shift_to_grab1'].Actions[-1].Condition= l.shift_to_grab1.Check
  sm['shift_to_grab1'].Actions[-1].Action= l.shift_to_grab1.Step
  sm['shift_to_grab1'].Actions[-1].NextState= ORIGIN_STATE
  sm['shift_to_grab1'].NewAction()
  sm['shift_to_grab1'].Actions[-1].Condition= lambda: IsFailure(l.shift_to_grab1.status)
  sm['shift_to_grab1'].Actions[-1].Action= lambda: ( sm.SetFailure(), l.shift_to_grab1.Exit() )
  sm['shift_to_grab1'].Actions[-1].NextState= EXIT_STATE
  sm['shift_to_grab1'].ElseAction.Condition= lambda: True
  sm['shift_to_grab1'].ElseAction.Action= l.shift_to_grab1.Exit
  sm['shift_to_grab1'].ElseAction.NextState= EXIT_STATE


  sm.Run()
  l= None

  return sm.ExitStatus
