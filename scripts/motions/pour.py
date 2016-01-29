#!/usr/bin/python
from core_tool import *
def Help():
  return '''Whole pouring procedure.
  Usage: pour BOTTLE_ID, CUP_ID [, AMOUNT_TRG [, MAX_DURATION [, CONSERVATIVE [, SPREADING [, TRICK]]]]]
    BOTTLE_ID: identifier of source bottle. e.g. 'b1'
    CUP_ID: identifier of receiving cup. e.g. 'c1'
    AMOUNT_TRG: Target amount (default=0.03)
    MAX_DURATION: Maximum duration (default=25.0)
    CONSERVATIVE: Robot becomes conservative, i.e. asking at each step (default=False)
    SPREADING: If True, robot spreads the material from source to receiver (default=False)
    TRICK: Select a trick to be used in flowc3_gen (default=None)
'''
def Run(t,*args):
  l= TContainer(debug=True)

  l.bottle= args[0]
  l.cup= args[1]
  l.amount_trg= args[2] if len(args)>2 else 0.03
  l.max_duration= args[3] if len(args)>3 else 25.0
  l.conservative= args[4] if len(args)>4 else False
  l.spreading= args[5] if len(args)>5 else False
  l.specified_trick= args[6] if len(args)>6 else None

  l.arm= LEFT
  l.lw_xe= t.GetAttr('wrist_'+LRToStrs(l.arm),'lx')

  l.timestamp= TimeStr('short2')
  l.sm_log_filen= t.LogFileName('pour',l.timestamp)
  l.sm_log_filep= file(l.sm_log_filen,'w')

  vislualization= True

  if vislualization:
    t.ExecuteMotion('viz', l.bottle,l.cup,'table','table0')


  t.kbhit.Activate()

  def ExitProc():
    t.kbhit.Deactivate()
    t.ar_adjust_ratio= t.default_ar_adjust_ratio
    t.DelAttr(l.bottle,'base_marker_id')
    t.DelAttr(l.cup,'base_marker_id')
    CPrint(1,'Logged pouring SM to',l.sm_log_filen)
    l.sm_log_filep.close()

    curr_filen= t.LogFileName('pourCond',l.timestamp,'.yaml')
    SaveYAML(t.GetAttr(CURR), curr_filen, lambda y:None)
    CPrint(1,'Logged pouring setup to',curr_filen)


  #Setup the pouring scene (registering base markers, adding a table)
  #TODO: change by arguments
  m_id_bottle= 1
  m_id_cup= 2
  auto_table= True
  t.ExecuteMotion('setup_pour_scene', l.bottle,l.cup,m_id_bottle,m_id_cup,auto_table)


  #FIXME: reconsider
  #Delete the previous temporary situation
  t.DelAttr(CURR)


  #Setup the collision check scene
  t.SetAttr(CURR,'scene', [l.bottle,l.cup,'table'])

  #Keep the initial pose to go back
  t.SetAttr(CURR,'init_x', t.robot.FK(x_ext=l.lw_xe, arm=l.arm))
  t.SetAttr(CURR,'init_handid', l.arm)

  #Setup pose estimation scene
  if t.GetAttr('environment')=='sim':
    t.SetAttr(CURR,'rtposeest_mode', '')
  else:
    #t.SetAttr(CURR,'rtposeest_mode', 'xtion')  #xtion only
    t.SetAttr(CURR,'rtposeest_mode', 'cmb')  #Combine xtion+m100
  if t.GetAttr(CURR,'rtposeest_mode') in ('xtion','cmb'):
    #Constrain the containers pose to be upright:
    t.ExecuteMotion('chx', l.bottle, 'upright')
    t.ExecuteMotion('chx', l.cup, 'upright')
    t.ExecuteMotion('rtposeest', 'clear', 'ext')
    t.ExecuteMotion('rtposeest', 'create', 'ext', 'sceneb', [l.bottle])
    t.ExecuteMotion('rtposeest', 'create', 'ext', 'scenec', [l.cup])
    t.ExecuteMotion('rtposeest', 'once', 'ext', l.bottle, 5, 'lin2dxy')
    t.ExecuteMotion('rtposeest', 'once', 'ext', l.cup, 5, 'lin2dxy')
    if t.GetAttr(CURR,'rtposeest_mode')=='cmb':
      t.SetAttr(CURR,'conservative', l.conservative)
      #CPrint(4,'grposeest is disabled...')
      #time.sleep(1.0)
      #Table height adjustment with a small cube model using m100:
      l.exec_status= t.ExecuteMotion('grposeest', ['table0'],'lin2dxz')
      xt= t.GetAttr('table','x')
      xt[2]= t.GetAttr('table0','x')[2]  #Update height
      t.SetAttr('table','x', xt)
      #Constrain the containers height on the table:
      t.ExecuteMotion('chx', l.bottle, 'ontable')
      t.ExecuteMotion('chx', l.cup, 'ontable')
      #Container pose adjustment with m100:
      l.exec_status= t.ExecuteMotion('grposeest', [l.bottle,l.cup], 'lin2dxy')  #'xyz'
      if IsFailure(l.exec_status):  return l.exec_status

  #Turn off x_sensor calibration with AR marker
  t.ar_adjust_ratio= 0.0
  #RGB-D sensor calibration using pose estimation of gripper with ray tracing pose estimator.
  #if t.GetAttr(CURR,'rtposeest_mode')=='xtion':
    #t.ExecuteMotion('rtcalib_x','wrist_l')

  #Describing current situation
  t.SetAttr(CURR,'task', 'pour')
  t.SetAttr(CURR,'taskid', 'pour%s'%l.timestamp)
  t.SetAttr(CURR,'source', l.bottle)
  t.SetAttr(CURR,'receiver', l.cup)
  t.SetAttr(CURR,'handid', l.arm)
  t.SetAttr(CURR,'conservative', l.conservative)

  t.ExecuteMotion('infer', l.bottle,'x')
  t.ExecuteMotion('infer', l.cup,'x')
  t.SetAttr(CURR,'init','source_x', t.GetAttr(l.bottle,'x'))
  t.SetAttr(CURR,'init','receiver_x', t.GetAttr(l.cup,'x'))
  t.SetAttr(CURR,'init','r_q', t.robot.Q(RIGHT))
  t.SetAttr(CURR,'init','l_q', t.robot.Q(LEFT))

  #Check the initial condition:
  t.ExecuteMotion('scene', 'make')
  isvalid, res= t.ExecuteMotion('scene','isvalidq',l.arm,[],t.robot.Q(l.arm),True)
  t.ExecuteMotion('scene', 'clear')
  if not isvalid:
    print 'Initial state is invalid.'
    print 'Colliding objects:',GetCollidingObjects(res.contacts)
    ExitProc()
    return FAILURE_PRECOND


  def AssessGrab(a, event_type, context, sm):
    MemorizeAsBad= lambda assess_key, assess_info, ask=True: \
        t.MemorizeAssessment(a, mem_key=['grab','bad'], assess_key=assess_key, assess_info=assess_info, context=copy.deepcopy(context), ask=ask)

    exception_req= False
    if t.GetAttrOr(False, CURR,'human_assessed_bad') and not a.MarkedBad:
      CPrint(3, 'Was the inference "grab" wrong?')
      if t.AskYesNo():
        if MemorizeAsBad(assess_key='human_assess', assess_info={'message':'bad'}, ask=False):
          exception_req= True
        #l.exec_status= FailureCode('human_assessed')

    if event_type==a.EVENT_EXIT and context==a.RegisteredContext[:-1]+['pregrab']:
      if not IsSuccess(sm.ExitStatus):
        MemorizeAsBad(assess_key='auto', assess_info={'kind':sm.ExitStatus})
      a.Conditions-= set(['pregrab'])
    elif event_type==a.EVENT_EXIT and context==a.RegisteredContext[:-1]+['grab']:
      if not IsSuccess(sm.ExitStatus):
        MemorizeAsBad(assess_key='auto', assess_info={'kind':sm.ExitStatus})
      else:
        arm= t.GetAttr(CURR,'handid')
        t.ExecuteMotion('scene', 'make',[l.bottle,'table'])  #Ignoring l.bottle and 'table'
        isvalid, vres= t.ExecuteMotion('scene','isvalidq',arm,[],t.robot.Q(arm),True)
        t.ExecuteMotion('scene', 'clear')
        if not isvalid:
          print 'In AssessGrab:'
          print 'Colliding objects:',GetCollidingObjects(vres.contacts)
          if MemorizeAsBad(assess_key='auto', assess_info={'kind':'invalid_state'}):
            exception_req= True
      a.Conditions-= set(['grab'])
    elif event_type==a.EVENT_EXIT and context==a.RegisteredContext[:-1]:
      a.Conditions= set()

    if t.GetAttrOr(False, CURR,'human_assessed_bad') and not exception_req:
      CPrint(3, 'Stop the motion now? (EXCEPTION)')
      if t.AskYesNo():  exception_req= True

    if exception_req:  return a.EXCEPTION
    return a.COMPLETED if len(a.Conditions)==0 else a.CONTINUE

  def InferGrab():
    #if t.GetAttr(CURR,'rtposeest_mode')=='xtion':
      #t.ExecuteMotion('rtposeest', 'once', 'ext', t.GetAttr(CURR,'source'), 3,'lin2dxy')
    situation= {
      'source':    t.GetAttr(CURR,'source'),
      'receiver':  t.GetAttr(CURR,'receiver'),
      'task':      t.GetAttr(CURR,'task'),
      'handid':    t.GetAttr(CURR,'handid')}
    inferred_keys_tmp= []
    if not t.ExecuteMotion('infer2', situation,[],'grab',inferred_keys_tmp):
      print 'Failed to infer grab parameters'
      l.exec_status= FailureCode('InferGrab')
      return
    inferred_keys= t.GetAttrOr([], CURR,'inferred_keys') + inferred_keys_tmp
    t.SetAttr(CURR,'inferred_keys', inferred_keys)

    assess_grab= TAssessment(TAssessment.STATE_CALLBACK, name='assess_grab')
    assess_grab.Assess= AssessGrab
    assess_grab.Conditions= set(['pregrab','grab'])
    assess_grab.DBIndex= t.GetAttr(CURR,'new_in_database')[-1]
    assess_grab.MarkedBad= False
    t.RegisterAssessment(assess_grab)
    l.exec_status= SUCCESS_CODE

  def AssessPour(a, event_type, context, sm):
    MemorizeAsBad= lambda assess_key, assess_info, ask=True: \
        t.MemorizeAssessment(a, mem_key=['pour','bad'], assess_key=assess_key, assess_info=assess_info, context=copy.deepcopy(context), ask=ask)

    exception_req= False
    if t.GetAttrOr(False, CURR,'human_assessed_bad') and not a.MarkedBad:
      CPrint(3, 'Was the inference "pour" wrong?')
      if t.AskYesNo():
        if MemorizeAsBad(assess_key='human_assess', assess_info={'message':'bad'}, ask=False):
          exception_req= True

    if event_type==a.EVENT_EXIT and context==a.RegisteredContext[:-1]+['prepour']:
      if not IsSuccess(sm.ExitStatus):
        MemorizeAsBad(assess_key='auto', assess_info={'kind':sm.ExitStatus})
      a.Conditions-= set(['prepour'])
    elif event_type==a.EVENT_EXIT and context==a.RegisteredContext[:-1]+['flowc_gen']:
      if not IsSuccess(sm.ExitStatus):
        MemorizeAsBad(assess_key='auto', assess_info={'kind':sm.ExitStatus})
      a.Conditions-= set(['flowc_gen'])
    elif event_type==a.EVENT_EXIT and context==a.RegisteredContext[:-1]+['postpour']:
      if not IsSuccess(sm.ExitStatus):
        MemorizeAsBad(assess_key='auto', assess_info={'kind':sm.ExitStatus})
      a.Conditions-= set(['postpour'])
    elif event_type==a.EVENT_EXIT and context==a.RegisteredContext[:-1]:
      a.Conditions= set()

    if t.GetAttrOr(False, CURR,'human_assessed_bad') and not exception_req:
      CPrint(3, 'Stop the motion now? (EXCEPTION)')
      if t.AskYesNo():  exception_req= True

    if exception_req:  return a.EXCEPTION
    return a.COMPLETED if len(a.Conditions)==0 else a.CONTINUE

  def InferPour():
    if t.GetAttr(CURR,'rtposeest_mode')=='xtion':
      t.ExecuteMotion('rtposeest', 'once', 'ext', t.GetAttr(CURR,'receiver'), 3,'lin2dxy')
    situation= {
      'source':    t.GetAttr(CURR,'source'),
      'receiver':  t.GetAttr(CURR,'receiver'),
      'task':      t.GetAttr(CURR,'task'),
      'handid':    t.GetAttr(CURR,'handid')}
    if 'dpl' in t.callback and t.callback.dpl!=None:
      l.situation= situation
      t.callback.dpl(t,l,'infer_pour')
      l.situation= None
    inferred_keys_tmp= []
    if not t.ExecuteMotion('infer2', situation,[],'pour',inferred_keys_tmp):
      print 'Cannot estimate l_x_pour_e of',l.bottle
      print '  and/or l_x_pour_l of',l.cup
      l.exec_status= FailureCode('InferPour')
      return
    inferred_keys= t.GetAttrOr([], CURR,'inferred_keys') + inferred_keys_tmp
    t.SetAttr(CURR,'inferred_keys', inferred_keys)

    assess_pour= TAssessment(TAssessment.STATE_CALLBACK, name='assess_pour')
    assess_pour.Assess= AssessPour
    assess_pour.Conditions= set(['prepour','flowc_gen','postpour'])
    assess_pour.DBIndex= t.GetAttr(CURR,'new_in_database')[-1]
    assess_pour.MarkedBad= False
    t.RegisterAssessment(assess_pour)
    l.exec_status= SUCCESS_CODE

  def InferSpreading():
    situation= {
      'receiver':  t.GetAttr(CURR,'receiver'),
      'task':      t.GetAttr(CURR,'task')}
    inferred_keys_tmp= []
    if not t.ExecuteMotion('infer2', situation,[],'spread',inferred_keys_tmp):
      print 'Cannot execute infer spread for ',t.GetAttr(CURR,'receiver')
      l.exec_status= FailureCode('InferSpreading')
      return
    inferred_keys= t.GetAttrOr([], CURR,'inferred_keys') + inferred_keys_tmp
    t.SetAttr(CURR,'inferred_keys', inferred_keys)

    t.SetAttr(CURR,'spread_start', list(t.GetAttr(CURR,'spread','xy_start'))+[t.GetAttr(CURR,'spread','z_max')])

    situation= {
      'source':    t.GetAttr(CURR,'source'),
      'receiver':  t.GetAttr(CURR,'receiver'),
      'task':      t.GetAttr(CURR,'task'),
      'handid':    t.GetAttr(CURR,'handid'),
      'spread_start': t.GetAttr(CURR,'spread_start')}
    inferred_keys_tmp= []
    if not t.ExecuteMotion('infer2', situation,[],'poursp',inferred_keys_tmp):
      print 'Cannot execute infer poursp for ',t.GetAttr(CURR,'source'),t.GetAttr(CURR,'receiver')
      l.exec_status= FailureCode('InferSpreading')
      return
    inferred_keys= t.GetAttrOr([], CURR,'inferred_keys') + inferred_keys_tmp
    t.SetAttr(CURR,'inferred_keys', inferred_keys)

    #assess_pour= TAssessment(TAssessment.STATE_CALLBACK, name='assess_pour')
    #assess_pour.Assess= AssessPour
    #assess_pour.Conditions= set(['prepour','flowc_gen','postpour'])
    #assess_pour.DBIndex= t.GetAttr(CURR,'new_in_database')[-1]
    #assess_pour.MarkedBad= False
    #t.RegisterAssessment(assess_pour)
    l.exec_status= SUCCESS_CODE


  #def @@@@@():
    #CPrint(3, 'Next motion: @@@@@')
    #if l.conservative and not t.AskYesNo():
      #l.exec_status= FailureCode('canceled')
    #else:
      #l.exec_status= @@@@@

  def PreGrab():
    if t.GetAttr(CURR,'rtposeest_mode')=='xtion':
      t.ExecuteMotion('rtposeest', 'run', 'ext', [t.GetAttr(CURR,'source')],['lin2dxy'])
    CPrint(3, 'Next motion: pregrab')
    if l.conservative and not t.AskYesNo():
      l.exec_status= FailureCode('canceled')
    else:
      l.exec_status= t.ExecuteMotion('pregrab', l.bottle,'l')
    if t.GetAttr(CURR,'rtposeest_mode')=='xtion':
      t.ExecuteMotion('rtposeest', 'stop', 'ext')

  def Grab():
    CPrint(3, 'Next motion: grab')
    if l.conservative and not t.AskYesNo():
      l.exec_status= FailureCode('canceled')
    else:
      l.exec_status= t.ExecuteMotion('grab', l.bottle,'l')

  def ReGrab():
    if 'regrab_assessment' in l:
      #If this regrab is second time (or more), asses the previous as bad
      t.MemorizeAssessment(l.regrab_assessment, mem_key=['regrab','bad'], assess_key='auto', assess_info={'kind':'regrab_repeated'}, context=t.state_context, ask=True)

    CPrint(3, 'Next motion: regrab')
    if l.conservative and not t.AskYesNo():
      l.exec_status= FailureCode('canceled')
    else:
      l.exec_status= t.ExecuteMotion('regrab', l.bottle)
      if IsSuccess(l.exec_status):
        l.regrab_assessment= TAssessment(0)  #callback_type, name are not important here
        l.regrab_assessment.DBIndex= t.ExecuteMotion('db', 'search',lambda s,d,a: s['infer_type']=='regrab')[0]
        l.regrab_assessment.MarkedBad= False

  def PrePour():
    CPrint(3, 'Next motion: prepour')
    if l.conservative and not t.AskYesNo():
      l.exec_status= FailureCode('canceled')
    else:
      l.exec_status= t.ExecuteMotion('prepour', l.bottle,l.cup)

  def FlowCGen():
    CPrint(3, 'Next motion: flowc_gen')
    if l.conservative and not t.AskYesNo():
      l.exec_status= FailureCode('canceled')
    else:
      l.exec_status= t.ExecuteMotion('flowc3_gen', l.bottle,l.cup,l.amount_trg,l.max_duration,l.specified_trick)
      if not IsSuccess(l.exec_status):
        time.sleep(3.0)

  def FlowCSpread():
    CPrint(3, 'Next motion: flowc_spread')
    if l.conservative and not t.AskYesNo():
      l.exec_status= FailureCode('canceled')
    else:
      l.exec_status= t.ExecuteMotion('flowc3_spread', l.bottle,l.cup,l.amount_trg,l.max_duration)
      if not IsSuccess(l.exec_status):
        time.sleep(3.0)

  def PostPour():
    CPrint(3, 'Next motion: postpour')
    if l.conservative and not t.AskYesNo():
      l.exec_status= FailureCode('canceled')
    else:
      l.exec_status= t.ExecuteMotion('postpour', l.bottle,l.cup)

  def Release():
    CPrint(3, 'Next motion: release')
    if l.conservative and not t.AskYesNo():
      l.exec_status= FailureCode('canceled')
    else:
      l.exec_status= t.ExecuteMotion('release', l.bottle)

  def PostGrab():
    CPrint(3, 'Next motion: postgrab')
    if l.conservative and not t.AskYesNo():
      l.exec_status= FailureCode('canceled')
    else:
      l.exec_status= t.ExecuteMotion('postgrab', l.bottle,'l')



  #InferGrab()
  #PreGrab()
  #Grab()
  #InferPour()
  #PrePour()
  #FlowCGen()
  #PostPour()
  #Release()
  #PostGrab()

  #Assessment by human's key input
  def AssessByHuman(a, event_type, context, sm):
    if t.kbhit.IsActive():
      key= t.kbhit.KBHit()
      if key==' ':
        CPrint(3, 'Help: press "x" to assess the current motion as BAD')
        CPrint(3, '      press "-" to EXIT IMMEDIATELY')
      elif key=='x':
        CPrint(3, 'Current motion is assessed as BAD, will be stopped')
        t.SetAttr(CURR,'human_assessed_bad', True)
        #l.exec_status= FailureCode('human_assessed')
      elif key=='-':
        CPrint(3, 'EXIT IMMEDIATELY..')
        a.ExceptionReq= True

    if a.ExceptionReq:
      return a.EXCEPTION  #This is necessary to stop all thread
    if event_type==a.EVENT_EXIT and context==a.RegisteredContext[:-1]:
      return a.COMPLETED
    return a.CONTINUE

  def AddHumanAssessment():
    assess_by_human= TAssessment(
        TAssessment.TIMER_CALLBACK|TAssessment.STATE_CALLBACK,
        name='assess_by_human', time_step=0.1)
    assess_by_human.Assess= AssessByHuman
    assess_by_human.ExceptionReq= False
    t.RegisterAssessment(assess_by_human)


  def SMLogger(sm, event_type, state, action):
    none_or= lambda x: str(x) if x!=None else 'None'
    l.sm_log_filep.write('%f %s %s %s %s\n' % (
                          rospy.Time.now().to_nsec(),
                          EventCodeToStr(event_type),
                          none_or(state), none_or(action),
                          sm.ExitStatus ) )


  sm= TStateMachine()
  sm.EventCallback= lambda sm,et,st,ac: ( t.SMCallback(sm,et,st,ac), SMLogger(sm,et,st,ac) )
  sm.Debug= True

  human_interruption= TFSMConditionedAction()
  human_interruption.Condition= lambda: t.GetAttrOr(False, CURR,'human_assessed_bad')
  human_interruption.Action= sm.SetFailure
  human_interruption.NextState= EXIT_STATE

  sm.StartState= 'start'

  sm.NewState('start')
  sm['start'].EntryAction= AddHumanAssessment
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'infer_grab'

  #sm.NewState('@@@@')
  #sm['@@@@'].EntryAction= @@@@
  #sm['@@@@'].NewAction()
  #sm['@@@@'].Actions[-1]= human_interruption
  #sm['@@@@'].NewAction()
  #sm['@@@@'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  #sm['@@@@'].Actions[-1].NextState= '@@@@'
  #sm['@@@@'].ElseAction.Condition= lambda: True
  #sm['@@@@'].ElseAction.Action= sm.SetFailure
  #sm['@@@@'].ElseAction.NextState= EXIT_STATE

  sm.NewState('infer_grab')
  sm['infer_grab'].EntryAction= InferGrab
  sm['infer_grab'].NewAction()
  sm['infer_grab'].Actions[-1]= human_interruption
  sm['infer_grab'].NewAction()
  sm['infer_grab'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['infer_grab'].Actions[-1].NextState= 'pregrab'
  sm['infer_grab'].ElseAction.Condition= lambda: True
  sm['infer_grab'].ElseAction.Action= sm.SetFailure
  sm['infer_grab'].ElseAction.NextState= EXIT_STATE

  sm.NewState('pregrab')
  sm['pregrab'].EntryAction= PreGrab
  sm['pregrab'].NewAction()
  sm['pregrab'].Actions[-1]= human_interruption
  sm['pregrab'].NewAction()
  sm['pregrab'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['pregrab'].Actions[-1].NextState= 'grab'
  sm['pregrab'].ElseAction.Condition= lambda: True
  sm['pregrab'].ElseAction.Action= sm.SetFailure
  sm['pregrab'].ElseAction.NextState= EXIT_STATE

  sm.NewState('grab')
  sm['grab'].EntryAction= Grab
  sm['grab'].NewAction()
  sm['grab'].Actions[-1]= human_interruption
  sm['grab'].NewAction()
  sm['grab'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status) and (not l.spreading)
  sm['grab'].Actions[-1].NextState= 'infer_pour'
  sm['grab'].NewAction()
  sm['grab'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status) and l.spreading
  sm['grab'].Actions[-1].NextState= 'infer_spreading'
  sm['grab'].ElseAction.Condition= lambda: True
  sm['grab'].ElseAction.Action= sm.SetFailure
  sm['grab'].ElseAction.NextState= EXIT_STATE

  sm.NewState('infer_pour')
  sm['infer_pour'].EntryAction= InferPour
  sm['infer_pour'].NewAction()
  sm['infer_pour'].Actions[-1]= human_interruption
  sm['infer_pour'].NewAction()
  sm['infer_pour'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['infer_pour'].Actions[-1].NextState= 'prepour'
  sm['infer_pour'].NewAction()
  sm['infer_pour'].Actions[-1].Condition= lambda:IsFailure(l.exec_status,'InferPour')
  sm['infer_pour'].Actions[-1].NextState= 'regrab'
  sm['infer_pour'].ElseAction.Condition= lambda: True
  sm['infer_pour'].ElseAction.Action= sm.SetFailure
  sm['infer_pour'].ElseAction.NextState= EXIT_STATE

  sm.NewState('infer_spreading')
  sm['infer_spreading'].EntryAction= InferSpreading
  sm['infer_spreading'].NewAction()
  sm['infer_spreading'].Actions[-1]= human_interruption
  sm['infer_spreading'].NewAction()
  sm['infer_spreading'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['infer_spreading'].Actions[-1].NextState= 'prepour'
  sm['infer_spreading'].NewAction()
  sm['infer_spreading'].Actions[-1].Condition= lambda:IsFailure(l.exec_status,'InferSpreading')
  sm['infer_spreading'].Actions[-1].NextState= 'regrab'
  sm['infer_spreading'].ElseAction.Condition= lambda: True
  sm['infer_spreading'].ElseAction.Action= sm.SetFailure
  sm['infer_spreading'].ElseAction.NextState= EXIT_STATE

  sm.NewState('regrab')
  sm['regrab'].EntryAction= ReGrab
  sm['regrab'].NewAction()
  sm['regrab'].Actions[-1]= human_interruption
  sm['regrab'].NewAction()
  sm['regrab'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['regrab'].Actions[-1].NextState= 'infer_grab'
  sm['regrab'].ElseAction.Condition= lambda: True
  sm['regrab'].ElseAction.Action= sm.SetFailure
  sm['regrab'].ElseAction.NextState= EXIT_STATE

  sm.NewState('prepour')
  sm['prepour'].EntryAction= PrePour
  sm['prepour'].NewAction()
  sm['prepour'].Actions[-1]= human_interruption
  sm['prepour'].NewAction()
  sm['prepour'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status) and (not l.spreading)
  sm['prepour'].Actions[-1].NextState= 'flowc_gen'
  sm['prepour'].NewAction()
  sm['prepour'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status) and l.spreading
  sm['prepour'].Actions[-1].NextState= 'flowc_spread'
  sm['prepour'].ElseAction.Condition= lambda: True
  sm['prepour'].ElseAction.Action= sm.SetFailure
  sm['prepour'].ElseAction.NextState= EXIT_STATE

  sm.NewState('flowc_gen')
  sm['flowc_gen'].EntryAction= FlowCGen
  sm['flowc_gen'].NewAction()
  sm['flowc_gen'].Actions[-1]= human_interruption
  sm['flowc_gen'].NewAction()
  sm['flowc_gen'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['flowc_gen'].Actions[-1].NextState= 'postpour'
  sm['flowc_gen'].NewAction()
  sm['flowc_gen'].Actions[-1].Condition= lambda:l.exec_status==FAILURE_PRECOND
  sm['flowc_gen'].Actions[-1].NextState= 'postpour'
  sm['flowc_gen'].ElseAction.Condition= lambda: True
  sm['flowc_gen'].ElseAction.Action= sm.SetFailure
  sm['flowc_gen'].ElseAction.NextState= EXIT_STATE

  sm.NewState('flowc_spread')
  sm['flowc_spread'].EntryAction= FlowCSpread
  sm['flowc_spread'].NewAction()
  sm['flowc_spread'].Actions[-1]= human_interruption
  sm['flowc_spread'].NewAction()
  sm['flowc_spread'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['flowc_spread'].Actions[-1].NextState= 'postpour'
  sm['flowc_spread'].NewAction()
  sm['flowc_spread'].Actions[-1].Condition= lambda:l.exec_status==FAILURE_PRECOND
  sm['flowc_spread'].Actions[-1].NextState= 'postpour'
  sm['flowc_spread'].ElseAction.Condition= lambda: True
  sm['flowc_spread'].ElseAction.Action= sm.SetFailure
  sm['flowc_spread'].ElseAction.NextState= EXIT_STATE

  sm.NewState('postpour')
  sm['postpour'].EntryAction= PostPour
  sm['postpour'].NewAction()
  sm['postpour'].Actions[-1]= human_interruption
  sm['postpour'].NewAction()
  sm['postpour'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['postpour'].Actions[-1].NextState= 'release'
  sm['postpour'].ElseAction.Condition= lambda: True
  sm['postpour'].ElseAction.Action= sm.SetFailure
  sm['postpour'].ElseAction.NextState= EXIT_STATE

  sm.NewState('release')
  sm['release'].EntryAction= Release
  sm['release'].NewAction()
  sm['release'].Actions[-1]= human_interruption
  sm['release'].NewAction()
  sm['release'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['release'].Actions[-1].NextState= 'postgrab'
  sm['release'].ElseAction.Condition= lambda: True
  sm['release'].ElseAction.Action= sm.SetFailure
  sm['release'].ElseAction.NextState= EXIT_STATE

  sm.NewState('postgrab')
  sm['postgrab'].EntryAction= PostGrab
  sm['postgrab'].NewAction()
  sm['postgrab'].Actions[-1]= human_interruption
  sm['postgrab'].NewAction()
  sm['postgrab'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['postgrab'].Actions[-1].NextState= EXIT_STATE
  sm['postgrab'].ElseAction.Condition= lambda: True
  sm['postgrab'].ElseAction.Action= sm.SetFailure
  sm['postgrab'].ElseAction.NextState= EXIT_STATE


  t.RunSM(sm,'pour')

  if IsSuccess(sm.ExitStatus):
    inferred_keys= t.GetAttrOr([], CURR,'inferred_keys')
    CPrint(3, 'Deleting:',inferred_keys)
    if not l.conservative or t.AskYesNo():
      for keys in inferred_keys:
        t.DelAttr(*keys)
    #t.DelAttr(CURR,'inferred_keys')

  if len(t.assessment_list)<>0:
    CPrint(4, 'Warning: There are still assessment_list:',t.assessment_list)
    CPrint(4, '  Check the assessment conditions.')
    #CPrint(4, 'Erase manually?')
    #if t.AskYesNo():
    t.RemoveAllAssessments()
  if len(t.state_context)<>0:
    CPrint(4, 'Warning: Context is not empty:',t.state_context)
    CPrint(4, '  Will be removed.')
    #CPrint(4, 'Erase manually?')
    #if t.AskYesNo():
    t.state_context= []

  ExitProc()
  l= None

  return sm.ExitStatus


