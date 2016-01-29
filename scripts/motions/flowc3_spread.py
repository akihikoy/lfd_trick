#!/usr/bin/python
from core_tool import *
def Help():
  return '''Flow controller for spreading (ver.3).
  Assumptions:
    Gripper holds a bottle
    Bottle is close to the cup
  Usage: flowc3_spread BOTTLE_ID, CUP_ID [, AMOUNT_TRG [, MAX_DURATION [, TRICK [, PARAM_ADJUST_D [, PARAM_ADJUST_C [, REPEAT]]]]]]
    BOTTLE_ID: Identifier of bottle. e.g. 'b1'
    CUP_ID: identifier of receiving cup. e.g. 'c1'
    AMOUNT_TRG: Target amount (default=0.8)  #NOT IMPORTANT
    MAX_DURATION: Maximum duration (default=60.0)
    TRICK: Select A trick to be used (default=None)
    PARAM_ADJUST_D: Using discrete parameter adjustment architecture, i.e. trick selection (default=True)
    PARAM_ADJUST_C: Using continuous parameter adjustment architecture (default=True)
    REPEAT: Repeat pouring, useful for parameter learning repeatedly (default=False)
    '''
def Run(t,*args):
  bottle= args[0]
  cup= args[1]
  amount_trg= args[2] if len(args)>2 else 0.8
  max_duration= args[3] if len(args)>3 else 60.0
  specified_trick= args[4] if len(args)>4 else None
  updating_param_d= args[5] if len(args)>5 else True
  updating_param_c= args[6] if len(args)>6 else True
  repeat= args[7] if len(args)>7 else False

  flow_trg= 0.05  #FIXME

  m_flowc_cmn= t.LoadMotion('flowc3_cmn')

  sm= TStateMachine(debug=True, local_obj=m_flowc_cmn.TLocal(debug=True))
  sm.EventCallback= t.SMCallback

  sm.l.specified_trick= specified_trick
  sm.l.updating_param_d= updating_param_d
  sm.l.updating_param_c= updating_param_c

  if not sm.l.Setup(t, bottle,cup,amount_trg,max_duration,flow_trg):
    return FAILURE_PRECOND

  sm.l.sub_sm= TContainer(debug=True)
  sm.l.sub_sm.std_pour= m_flowc_cmn.GenSMStdPour(t,sm.l)
  #sm.l.sub_sm.shake_A= m_flowc_cmn.GenSMShakeA(t,sm.l)
  #sm.l.sub_sm.shake_B= m_flowc_cmn.GenSMShakeB(t,sm.l)

  sm.l.sub_sm.height_ctrl= m_flowc_cmn.GenSMPourHeightCtrl(t,sm.l)
  sm.l.sub_sm.spread= m_flowc_cmn.GenSMSpreadCtrl3(t,sm.l)

  def get_trick_id():
    return 'std_pour'

  def SetBehavior(b):
    sm.l.behavior_type= b

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= 'stop'

  sm.StartState= 'init'
  sm.NewState('init')
  sm['init'].EntryAction= lambda: ( t.RunSMAsThread(sm.l.sub_sm.spread,'flowc_spread'),
                                    t.RunSMAsThread(sm.l.sub_sm.height_ctrl,'flowc_height_ctrl') )
  sm['init'].ElseAction.Condition= lambda: True
  sm['init'].ElseAction.NextState= 'start'

  sm.NewState('start')
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= sm.l.IsPoured
  sm['start'].Actions[-1].NextState= 'stop'
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: sm.l.is_spreaded
  sm['start'].Actions[-1].NextState= 'stop'
  sm['start'].NewAction()
  sm['start'].Actions[-1]= timeout_action
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: get_trick_id()=='std_pour'
  sm['start'].Actions[-1].Action= lambda: SetBehavior(get_trick_id())
  sm['start'].Actions[-1].NextState= 'std_pour'

  sm.NewState('std_pour')
  sm['std_pour'].EntryAction= lambda: t.RunSM(sm.l.sub_sm.std_pour,'flowc_std_pour')
  sm['std_pour'].ElseAction.Condition= lambda: True
  sm['std_pour'].ElseAction.NextState= 'start'

  sm.NewState('stop')
  sm['stop'].NewAction()
  sm['stop'].Actions[-1].Condition= lambda: sm.l.IsThetaEqTo(0.0)
  sm['stop'].Actions[-1].Action= lambda: ( sm.l.sub_sm.height_ctrl.ThreadInfo.Stop(), Print('End of spreading') )
  sm['stop'].Actions[-1].NextState= EXIT_STATE
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.Action= lambda: sm.l.ControlStep(sm.l.flow_control_dtheta_min)
  sm['stop'].ElseAction.NextState= ORIGIN_STATE

  #sm.Show()
  t.RunSM(sm,'flowc3_spread')
  sm.l.Close()

  for sub_sm in sm.l.sub_sm.values():
    sub_sm.Cleanup()
  sm.Cleanup()

  return sm.ExitStatus
