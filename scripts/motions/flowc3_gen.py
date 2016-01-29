#!/usr/bin/python
from core_tool import *
def Help():
  return '''General flow amount controller (ver.3).
  Assumptions:
    Gripper holds a bottle
    Bottle is close to the cup
  Usage: flowc3_gen BOTTLE_ID, CUP_ID [, AMOUNT_TRG [, MAX_DURATION [, TRICK [, PARAM_ADJUST_D [, PARAM_ADJUST_C [, REPEAT]]]]]]
    BOTTLE_ID: Identifier of bottle. e.g. 'b1'
    CUP_ID: identifier of receiving cup. e.g. 'c1'
    AMOUNT_TRG: Target amount (default=0.03)
    MAX_DURATION: Maximum duration (default=25.0)
    TRICK: Select A trick to be used (default=None)
    PARAM_ADJUST_D: Using discrete parameter adjustment architecture, i.e. trick selection (default=True)
    PARAM_ADJUST_C: Using continuous parameter adjustment architecture (default=True)
    REPEAT: Repeat pouring, useful for parameter learning repeatedly (default=False)
    '''
def Run(t,*args):
  bottle= args[0]
  cup= args[1]
  amount_trg= args[2] if len(args)>2 else 0.03
  max_duration= args[3] if len(args)>3 else 25.0
  specified_trick= args[4] if len(args)>4 else None
  updating_param_d= args[5] if len(args)>5 else True
  updating_param_c= args[6] if len(args)>6 else True
  repeat= args[7] if len(args)>7 else False

  flow_trg= 0.05  #FIXME

  m_flowc_cmn= t.LoadMotion('flowc3_cmn')

  sm= TStateMachine(debug=True, local_obj=m_flowc_cmn.TLocal(debug=True))  #local_obj can be accessed with sm.l
  sm.EventCallback= t.SMCallback

  sm.l.specified_trick= specified_trick
  sm.l.updating_param_d= updating_param_d
  sm.l.updating_param_c= updating_param_c

  if not sm.l.Setup(t, bottle,cup,amount_trg,max_duration,flow_trg):
    return FAILURE_PRECOND

  #sm.l.shake_widthB_ext will be planned.
  if 'dpl' in t.callback and t.callback.dpl!=None:  t.callback.dpl(t,sm.l,'before_flowc_gen')

  sm.l.sub_sm= TContainer(debug=True)
  sm.l.sub_sm.std_pour= m_flowc_cmn.GenSMStdPour(t,sm.l)
  sm.l.sub_sm.shake_A= m_flowc_cmn.GenSMShakeA(t,sm.l)
  sm.l.sub_sm.shake_B= m_flowc_cmn.GenSMShakeB(t,sm.l)

  sm.l.sub_sm.height_ctrl= m_flowc_cmn.GenSMPourHeightCtrl(t,sm.l)

  #TEST:TEST_SPREAD
  #sm.l.sub_sm.spread= m_flowc_cmn.GenSMSpreadCtrl3(t,sm.l)

  def SetBehavior(b):
    sm.l.behavior_type= b

  #Search the best trick_id
  sm.Params['trick_id']= TDiscParam()
  trick_id= sm.Params['trick_id']
  trick_id.Candidates= ['std_pour','shake_A','shake_B']
  trick_id.Means= t.GetAttrOr([1.0,0.5,0.5], 'memory','flowc',sm.l.bottle,'trick_id_means')
  trick_id.SqMeans= t.GetAttrOr([1.0+1.0,0.25+1.0,0.25+1.0], 'memory','flowc',sm.l.bottle,'trick_id_sqmeans')
  trick_id.Alpha= 0.4
  trick_id.Init()  #FIXME:Use d=Save() and Init(d) to save and restore the data
  sm.l.trick_id_can_be_updated= False
  def select_trick_id():
    sm.l.trick_id_amount_begin= sm.l.flow_sensor.amount
    sm.l.trick_id_time_begin= sm.l.elapsed_time
    sm.l.trick_id_can_be_updated= True
    trick_id.Select()
  def get_trick_id():
    if sm.l.specified_trick:
      return sm.l.specified_trick
    if not sm.l.updating_param_d:
      return 'std_pour'
    return trick_id.Param()
    #return 'shake_B'
  def update_trick_id():
    if sm.l.updating_param_d and sm.l.trick_id_can_be_updated:
      score= 100.0*(sm.l.flow_sensor.amount - sm.l.trick_id_amount_begin) / (sm.l.elapsed_time - sm.l.trick_id_time_begin)
      #score= (sm.l.flow_sensor.amount - sm.l.trick_id_amount_begin)
      trick_id.Update(score)
      sm.l.trick_id_can_be_updated= False
      #print '###DEBUG:',sm.l.trick_id_amount_begin,sm.l.flow_sensor.amount, '@',sm.l.elapsed_time - sm.l.trick_id_time_begin
      #print '###DEBUG'; t.AskYesNo()


  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= 'stop'

  sm.StartState= 'init'
  sm.NewState('init')
  sm['init'].EntryAction= lambda: t.RunSMAsThread(sm.l.sub_sm.height_ctrl,'flowc_height_ctrl')
  #sm['init'].EntryAction= lambda: ( t.RunSMAsThread(sm.l.sub_sm.height_ctrl,'flowc_height_ctrl'),
                                   #t.RunSMAsThread(sm.l.sub_sm.spread,'flowc_spread') )  #TEST:TEST_SPREAD
  sm['init'].ElseAction.Condition= lambda: True
  sm['init'].ElseAction.NextState= 'start'

  sm.NewState('start')
  sm['start'].EntryAction= lambda: ( update_trick_id(), select_trick_id() )
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= sm.l.IsPoured
  sm['start'].Actions[-1].NextState= 'stop'
  sm['start'].NewAction()
  sm['start'].Actions[-1]= timeout_action
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: get_trick_id()=='std_pour'
  sm['start'].Actions[-1].Action= lambda: SetBehavior(get_trick_id())
  sm['start'].Actions[-1].NextState= 'std_pour'
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: get_trick_id()=='shake_A'
  sm['start'].Actions[-1].Action= lambda: SetBehavior(get_trick_id())
  sm['start'].Actions[-1].NextState= 'shake_A'
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: get_trick_id()=='shake_B'
  sm['start'].Actions[-1].Action= lambda: SetBehavior(get_trick_id())
  sm['start'].Actions[-1].NextState= 'shake_B'

  sm.NewState('std_pour')
  sm['std_pour'].EntryAction= lambda: t.RunSM(sm.l.sub_sm.std_pour,'flowc_std_pour')
  sm['std_pour'].ElseAction.Condition= lambda: True
  sm['std_pour'].ElseAction.NextState= 'start'

  sm.NewState('shake_A')
  sm['shake_A'].EntryAction= lambda: t.RunSM(sm.l.sub_sm.shake_A,'flowc_shake_A')
  sm['shake_A'].ElseAction.Condition= lambda: True
  sm['shake_A'].ElseAction.NextState= 'start'

  sm.NewState('shake_B')
  sm['shake_B'].EntryAction= lambda: t.RunSM(sm.l.sub_sm.shake_B,'flowc_shake_B')
  sm['shake_B'].ElseAction.Condition= lambda: True
  sm['shake_B'].ElseAction.NextState= 'start'

  sm.NewState('stop')
  sm['stop'].NewAction()
  sm['stop'].Actions[-1].Condition= lambda: sm.l.IsThetaEqTo(0.0)
  sm['stop'].Actions[-1].Action= lambda: ( sm.l.sub_sm.height_ctrl.ThreadInfo.Stop(),
                                          Print('End of pouring'),
                                          Print('First pour: ',sm.l.first_poured_time) )
  sm['stop'].Actions[-1].NextState= EXIT_STATE
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.Action= lambda: sm.l.ControlStep(sm.l.flow_control_dtheta_min)
  sm['stop'].ElseAction.NextState= ORIGIN_STATE

  def ShowParam():
    for params in (sm.Params,
                   sm.l.sub_sm.std_pour.Params,
                   sm.l.sub_sm.shake_A.Params,
                   sm.l.sub_sm.shake_B.Params):
      for (key,param) in params.items():
        print 'Param ',key,':'
        if isinstance(param,TDiscParam):
          print '  Means:',param.Means
          print '  UCB:',param.UCB()
          print '  SqMeans:',param.SqMeans
        elif isinstance(param,TContParamGrad):
          print '  Mean:',param.Mean
        elif isinstance(param,TContParamNoGrad):
          print '  xopt:',param.es.result()[0]
          print '  fopt:',param.es.result()[1]
          print '  xmean:',param.es.result()[5]
          print '  stds:',param.es.result()[6]

  #sm.Show()
  t.RunSM(sm,'flowc3_gen')
  ShowParam()
  if 'dpl' in t.callback and t.callback.dpl!=None:  t.callback.dpl(t,sm.l,'after_flowc_gen')
  sm.l.Close()

  if repeat:
    n_trial= 1
    while IsSuccess(sm.ExitStatus):
      print 'Try again? (',n_trial,')'
      if not t.AskYesNo():
        break

      n_trial+= 1
      sm.l.Reset()
      sm.l.trick_id_can_be_updated= False
      sm.l.shake_axis_can_be_updated= False
      t.RunSM(sm,'flowc3_gen')
      ShowParam()
      sm.l.Close()

  for sub_sm in sm.l.sub_sm.values():
    sub_sm.Cleanup()
  sm.Cleanup()

  return sm.ExitStatus
