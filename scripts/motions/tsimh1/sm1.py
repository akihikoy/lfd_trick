#!/usr/bin/python
from core_tool import *
def Help():
  return '''State machine for ODE hopping robot simulation.
  Usage: tsimh1.sm1'''

#Move to pour point state machine
def PrePourSM(t,l,sim):
  def MoveToPourL0():
    p_pour0= copy.deepcopy(l.sensors.p_pour)
    theta0= l.sensors.theta
    p_pour_msg= std_msgs.msg.Float64MultiArray()
    theta_msg= std_msgs.msg.Float64()
    Ndiv= max(5, int(Dist(l.p_pour_trg0,p_pour0)*50))
    for tc in FRange1(0.0,1.0,Ndiv):
      p_pour_msg.data= (1.0-tc)*Vec(p_pour0) + tc*Vec(l.p_pour_trg0)
      theta_msg.data= (1.0-tc)*theta0 + tc*l.theta_init
      t.pub.ode_ppour.publish(p_pour_msg)
      t.pub.ode_theta.publish(theta_msg)
      sim.SimSleep(t,l,0.04)
      #if l.sensors.src_colliding or l.sensors.gripper_colliding:
        #break
    l.exec_status= SUCCESS_CODE

  sm= TStateMachine()
  sm.EventCallback= t.SMCallback
  sm.Debug= True

  sm.StartState= 'start'
  sm.NewState('start')
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'move_upward'

  sm.NewState('move_upward')
  sm['move_upward'].NewAction()
  sm['move_upward'].Actions[-1].Condition= lambda: l.sensors.src_colliding
  sm['move_upward'].Actions[-1].Action= lambda: sim.MoveDPPour(t,l,[0.0,0.0,0.01])
  sm['move_upward'].Actions[-1].NextState= ORIGIN_STATE
  sm['move_upward'].ElseAction.Condition= lambda: True
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
  sm['shift_to_pour_l'].EntryAction= lambda: setattr(l,'shift_to_pour_l_reached',False)
  sm['shift_to_pour_l'].NewAction()
  sm['shift_to_pour_l'].Actions[-1].Condition= lambda: not l.shift_to_pour_l_reached and not l.sensors.src_colliding and not l.sensors.gripper_colliding
  sm['shift_to_pour_l'].Actions[-1].Action= lambda: setattr(l,'shift_to_pour_l_reached',
                                                            sim.MoveToTrgPPour(t,l,l.p_pour_trg,spd=0.1))
  sm['shift_to_pour_l'].Actions[-1].NextState= ORIGIN_STATE
  sm['shift_to_pour_l'].ElseAction.Condition= lambda: True
  sm['shift_to_pour_l'].ElseAction.NextState= EXIT_STATE

  t.RunSM(sm,'prepour')
  l= None
  return sm.ExitStatus


def GenSMStdPour(t,l,sim):
  sm= TStateMachine(debug=True, local_obj=l)
  sm.EventCallback= t.SMCallback

  sm.l.theta_flowstart= 0.5*math.pi
  sm.l.max_theta= 0.9*math.pi

  poured_action= TFSMConditionedAction()
  poured_action.Condition= sm.l.IsPoured
  poured_action.NextState= EXIT_STATE

  spilled_action= TFSMConditionedAction()
  spilled_action.Condition= sm.l.IsSpilled
  spilled_action.NextState= EXIT_STATE

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= EXIT_STATE

  sm.StartState= 'to_initial'

  sm.NewState('to_initial')
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1]= poured_action
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1]= spilled_action
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1]= timeout_action
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1].Condition= lambda: sm.l.sensors.theta<=sm.l.theta_init
  sm['to_initial'].Actions[-1].NextState= 'approach'  #NEW_TFlowAmountModel
  sm['to_initial'].ElseAction.Condition= lambda: True
  sm['to_initial'].ElseAction.Action= lambda: sim.MoveDTheta(t,sm.l,-sm.l.dtheta_max)
  sm['to_initial'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('approach')  #NEW_TFlowAmountModel
  sm['approach'].NewAction()
  sm['approach'].Actions[-1]= poured_action
  sm['approach'].NewAction()
  sm['approach'].Actions[-1]= spilled_action
  sm['approach'].NewAction()
  sm['approach'].Actions[-1]= timeout_action
  sm['approach'].NewAction()
  sm['approach'].NewAction()
  sm['approach'].Actions[-1].Condition= lambda: sm.l.sensors.theta>sm.l.theta_flowstart
  sm['approach'].Actions[-1].NextState= 'find_flow_p'
  sm['approach'].ElseAction.Condition= lambda: True
  sm['approach'].ElseAction.Action= lambda: sim.MoveDTheta(t,sm.l,0.7*sm.l.dtheta_max)
  sm['approach'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('find_flow_p')
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= poured_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= spilled_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= timeout_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1].Condition= lambda: sm.l.sensors.num_flow>0
  sm['find_flow_p'].Actions[-1].NextState= 'pour'
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1].Condition= lambda: sm.l.sensors.theta>sm.l.max_theta
  sm['find_flow_p'].Actions[-1].NextState= EXIT_STATE
  sm['find_flow_p'].ElseAction.Condition= lambda: True
  sm['find_flow_p'].ElseAction.Action= lambda: sim.MoveDTheta(t,sm.l,0.2*sm.l.dtheta_max)
  sm['find_flow_p'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('pour')
  sm['pour'].EntryAction= lambda: sm.l.ChargeTimer(5.0)  #Time of patience
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= poured_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= spilled_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= timeout_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= lambda: sm.l.sensors.num_flow>0
  sm['pour'].Actions[-1].Action= lambda: ( sm.l.ChargeTimer(5.0), sim.MoveDTheta(t,sm.l,0.0) )
  sm['pour'].Actions[-1].NextState= ORIGIN_STATE
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= lambda: sm.l.sensors.time>sm.l.timer_tstop
  sm['pour'].Actions[-1].NextState= EXIT_STATE
  sm['pour'].ElseAction.Condition= lambda: True
  sm['pour'].ElseAction.Action= lambda: sim.MoveDTheta(t,sm.l,0.2*sm.l.dtheta_max)
  sm['pour'].ElseAction.NextState= ORIGIN_STATE

  return sm


def GenSMPourHeightCtrl(t,l,sim):
  sm= TStateMachine(debug=False, local_obj=l)
  #sm.EventCallback= t.SMCallback  #WARNING: No callback to avoide confuse the context

  poured_action= TFSMConditionedAction()
  poured_action.Condition= sm.l.IsPoured
  poured_action.NextState= 'stop'

  spilled_action= TFSMConditionedAction()
  spilled_action.Condition= sm.l.IsSpilled
  spilled_action.NextState= 'stop'

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= 'stop'

  sm.StartState= 'init'

  def Init():
    sm.l.p_pour_init= copy.deepcopy(sm.l.sensors.p_pour)

  def HeadCtrlStep():
    if sm.l.sensors.src_colliding or sm.l.sensors.gripper_colliding:
      #If collision
      sim.MoveToTrgPPour(t,sm.l,sm.l.p_pour_trg0,spd=0.06)
    else:
      sim.MoveToTrgPPour(t,sm.l,sm.l.p_pour_trg,spd=0.06)

  def StopStep():
    sim.MoveToTrgPPour(t,sm.l,sm.l.p_pour_init,spd=0.1)
    #if sm.l.sensors.src_colliding or sm.l.sensors.gripper_colliding:
      ##If collision
      #sim.MoveToTrgPPour(t,sm.l,sm.l.p_pour_trg0,spd=0.1)

  sm.NewState('init')
  sm['init'].EntryAction= Init
  sm['init'].ElseAction.Condition= lambda: True
  sm['init'].ElseAction.NextState= 'move'

  sm.NewState('move')
  sm['move'].NewAction()
  sm['move'].Actions[-1].Condition= lambda: not sm.ThreadInfo.IsRunning()
  sm['move'].Actions[-1].NextState= 'stop'
  sm['move'].NewAction()
  sm['move'].Actions[-1]= poured_action
  sm['move'].NewAction()
  sm['move'].Actions[-1]= spilled_action
  sm['move'].NewAction()
  sm['move'].Actions[-1]= timeout_action
  sm['move'].ElseAction.Condition= lambda: True
  sm['move'].ElseAction.Action= HeadCtrlStep
  sm['move'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('stop')
  sm['stop'].NewAction()
  sm['stop'].Actions[-1].Condition= lambda: sm.ThreadInfo.IsRunning()
  sm['stop'].Actions[-1].Action= StopStep
  sm['stop'].Actions[-1].NextState= ORIGIN_STATE
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.NextState= EXIT_STATE

  return sm


def ApplyFilter(t,l,sim):
  if 'num_spill' not in l.filtered:  l.filtered.num_spill= l.sensors.num_spill
  else:  l.filtered.num_spill= max(l.filtered.num_spill, l.sensors.num_spill)
  if 'num_bounce' not in l.filtered:  l.filtered.num_bounce= l.sensors.num_bounce
  else:  l.filtered.num_bounce= max(l.filtered.num_bounce, l.sensors.num_bounce)
  #l.filtered.amount= l.sensors.z_rcv*5.0 if l.sensors.z_rcv>0.0 else 0.0
  l.filtered.amount= 0.0055*l.sensors.num_rcv

  if 'v_rcv' not in l.filtered:
    l.filtered.v_rcv= [0.0, 0.0, 0.0]
    l.filtered.p_rcv_prev= [l.sensors.x_rcv.position.x, l.sensors.x_rcv.position.y, l.sensors.x_rcv.position.z]
  p_rcv= [l.sensors.x_rcv.position.x, l.sensors.x_rcv.position.y, l.sensors.x_rcv.position.z]
  l.filtered.v_rcv= [(p_rcv[d]-l.filtered.p_rcv_prev[d])/l.config.TimeStep for d in range(3)]

  ## Ball status kinds (1:in src, 2:in rcv, 3:flow, 4:spill, 0:unknown)
  #if 'ball_st' not in l.filtered:
    ##If only st==1 (in src) is in l.sensors.ball_st:
    #if [st for st in l.sensors.ball_st if st!=1]==[]:
      #l.filtered.ball_st= [1]*len(l.sensors.ball_st)
  #else:

  if l.flow_controlling:
    if 'term_flow_x' not in l.filtered:
      l.filtered.term_flow_x= []
      l.filtered.term_flow_center= [0.0,0.0,0.0]
      l.filtered.term_flow_var= 0.0
      l.filtered.term_flow_max_dist= 0.0
      l.filtered.obs_counter= 0
    #l.sensors.ball_st, l.sensors.ball_x
    flow_x= [l.sensors.ball_x[6*i:6*(i+1)] for i in range(len(l.sensors.ball_st)) if l.sensors.ball_st[i]==3]
    #Note: ball_st[i]==3 denotes a flow particle
    #flow_x.sort(key=lambda x:x[2])  #sort by z
    #N= 10
    #term_flow_x= flow_x[:N]  #particles on the floor or the bottom of the receiver
    l.filtered.term_flow_x+= [xf for xf in flow_x if xf[2]<0.1]  #particles whose height is < 10 cm
    random.shuffle(l.filtered.term_flow_x)
    N= 50
    l.filtered.term_flow_x= l.filtered.term_flow_x[:N]
    term_flow_x= l.filtered.term_flow_x
    N= len(term_flow_x)
    if N>0:
      #term_flow_x.sort(key=lambda x:x[2])  #sort by z
      term_flow_center= [sum([term_flow_x[i][0] for i in range(N)])/float(N),
                        sum([term_flow_x[i][1] for i in range(N)])/float(N),
                        #sum([term_flow_x[i][2] for i in range(N)])/float(N)
                        0.0 ]
      dist_x_data= [(Dist(term_flow_x[i][:3], term_flow_center), term_flow_x[i]) for i in range(N)]
      dist_x_data.sort(reverse=True, key=lambda x:x[0])
      term_flow_var= sum([d*d for d,x in dist_x_data]) / float(N)
      term_flow_max_dist= dist_x_data[0][0]
      #term_flow_center : center of flow
      #dist_x_data[0][0] : most far flow
      #dist_x_data[0][1] : corresponding point
      l.filtered.obs_counter+= 1
      if l.filtered.term_flow_center==[0.0,0.0,0.0]:
        l.filtered.term_flow_center= term_flow_center
      else:
        alpha= 0.5/(0.1*l.filtered.obs_counter+1.0)
        l.filtered.term_flow_center= [(1.0-alpha)*f0+alpha*f1 for f0,f1 in zip(l.filtered.term_flow_center,term_flow_center)]
      l.filtered.term_flow_var= min(max(l.filtered.term_flow_var, term_flow_var), 1.0)
      l.filtered.term_flow_max_dist= min(max(l.filtered.term_flow_max_dist, term_flow_max_dist), 1.0)

  #Visualize flow:
  if 'term_flow_max_dist' in l.filtered and l.filtered.term_flow_max_dist>0.0:
    msg= lfd_sim.msg.ODEViz()
    if 'user_viz' in l:
      msg.objects+= l.user_viz
    prm= lfd_sim.msg.ODEVizPrimitive()
    prm.type= prm.LINE
    prm.pose.position.x= l.filtered.term_flow_center[0]
    prm.pose.position.y= l.filtered.term_flow_center[1]
    prm.pose.position.z= l.filtered.term_flow_center[2]
    prm.param= [0.0,0.0,0.3]
    prm.color.r= 0.0
    prm.color.g= 1.0
    prm.color.b= 1.0
    prm.color.a= 0.2
    msg.objects.append(prm)
    prm= lfd_sim.msg.ODEVizPrimitive()
    prm.type= prm.CYLINDER
    prm.pose.position.x= l.filtered.term_flow_center[0]
    prm.pose.position.y= l.filtered.term_flow_center[1]
    prm.pose.position.z= l.filtered.term_flow_center[2]
    prm.param= [math.sqrt(l.filtered.term_flow_var), 0.015]
    prm.color.r= 1.0
    prm.color.g= 1.0
    prm.color.b= 0.0
    prm.color.a= 0.1
    msg.objects.append(prm)
    prm= lfd_sim.msg.ODEVizPrimitive()
    prm.type= prm.CYLINDER
    prm.pose.position.x= l.filtered.term_flow_center[0]
    prm.pose.position.y= l.filtered.term_flow_center[1]
    prm.pose.position.z= l.filtered.term_flow_center[2]
    prm.param= [l.filtered.term_flow_max_dist, 0.007]
    prm.color.r= 1.0
    prm.color.g= 1.0
    prm.color.b= 0.0
    prm.color.a= 0.1
    msg.objects.append(prm)
    t.pub.ode_viz.publish(msg)
  elif 'user_viz' in l:
    msg= lfd_sim.msg.ODEViz()
    msg.objects+= l.user_viz
    t.pub.ode_viz.publish(msg)


def FlowCGenSM(t,l,sim):
  sm= TStateMachine(debug=True)
  sm.EventCallback= t.SMCallback

  l.amount_trg= 0.3
  l.max_duration= 40.0
  l.flow_trg= 0.05  #FIXME
  l.dtheta_max= 0.02

  l.start_time= l.sensors.time
  l.IsTimeout= lambda: (l.sensors.time-l.start_time > l.max_duration)
  l.IsPoured= lambda: (l.filtered.amount > l.amount_trg)
  l.IsSpilled= lambda: (l.filtered.num_spill >= 5)
  l.ChargeTimer= lambda dt: setattr(l,'timer_tstop',l.sensors.time+dt)

  l.sub_sm= TContainer()
  l.sub_sm.std_pour= GenSMStdPour(t,l,sim)
  l.sub_sm.height_ctrl= GenSMPourHeightCtrl(t,l,sim)


  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= l.IsTimeout
  timeout_action.NextState= 'stop'

  sm.StartState= 'init'
  sm.NewState('init')
  sm['init'].EntryAction= lambda: t.RunSMAsThread(l.sub_sm.height_ctrl,'flowc_height_ctrl')
  sm['init'].ElseAction.Condition= lambda: True
  sm['init'].ElseAction.NextState= 'start'

  sm.NewState('start')
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= l.IsPoured
  sm['start'].Actions[-1].NextState= 'stop'
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= l.IsSpilled
  sm['start'].Actions[-1].NextState= 'stop'
  sm['start'].NewAction()
  sm['start'].Actions[-1]= timeout_action
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: True
  #sm['start'].Actions[-1].Action= lambda: SetBehavior('std_pour')
  sm['start'].Actions[-1].NextState= 'std_pour'

  sm.NewState('std_pour')
  sm['std_pour'].EntryAction= lambda: t.RunSM(l.sub_sm.std_pour,'flowc_std_pour')
  sm['std_pour'].ElseAction.Condition= lambda: True
  sm['std_pour'].ElseAction.NextState= 'start'

  sm.NewState('stop')
  sm['stop'].NewAction()
  sm['stop'].Actions[-1].Condition= lambda: l.sensors.theta<=l.theta_init
  sm['stop'].Actions[-1].Action= lambda: ( l.sub_sm.height_ctrl.ThreadInfo.Stop(),
                                          Print('End of pouring') )
  sm['stop'].Actions[-1].NextState= EXIT_STATE
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.Action= lambda: sim.MoveDTheta(t,l,-l.dtheta_max)
  sm['stop'].ElseAction.NextState= ORIGIN_STATE

  t.RunSM(sm,'flowc3_gen')

  for sub_sm in l.sub_sm.values():
    sub_sm.Cleanup()
  sm.Cleanup()

  return sm.ExitStatus


def PourSM(t,l):
  sim= t.LoadMotion('tsimh1.core1')

  def Log(msg):
    if 'sm_logfp' in l and l.sm_logfp is not None:  out= l.sm_logfp
    else:  out= sys.stdout
    out.write('-------------------\n')
    out.write('======== LOGENTRY: %s ========\n'%msg)
    out.write('<<<l.config>>>\n')
    out.write( str(l.config) )
    out.write('\n<<<l>>>\n')
    for key in ('p_pour_trg','p_pour_trg0','theta_init'):
      out.write('l.{key}: {x}\n'.format(key=key,x=l[key] if key in l else 'N/A'))
    out.write('-------------------\n')
    out.flush()

  def Setup():
    sim.SetupServiceProxy(t,l)
    sim.SetupPubSub(t,l)

    t.srvp.ode_resume()
    l.config= sim.GetConfig(t)
    print 'Current config:',l.config

    #Setup config
    l.config.MaxContacts= 2
    l.config.TimeStep= 0.025
    l.config.Gravity= -1.0
    l.config.BallType= 0  #Sphere particles
    #l.config.BallType= 1  #Box particles
    #Bounce balls:
    l.config.ContactBounce= 0.7
    l.config.ContactBounceVel= 0.2
    if 'config_callback' in l and l.config_callback!=None:
      l.config_callback(t,l,sim)
    Log('After l.config_callback')

    #Reset to get state for plan
    sim.ResetConfig(t,l.config)
    time.sleep(0.1)  #Wait for l.sensors is updated
    t.srvp.ode_pause()  #Pause to wait grasp plan

    l.filtered= TContainer(debug=True)
    l.flow_controlling= False
    l.sensor_callback= lambda:ApplyFilter(t,l,sim)  #Activate filter

  def InferGrab():
    #Plan grasp
    l.planlearn_callback(t,l,sim,'infer_grab')
    Log('After infer_grab')

  def Grab():
    t.srvp.ode_resume()
    #Reset again to apply the grasp plan
    sim.ResetConfig(t,l.config)
    Log('After sim.ResetConfig in Grab')
    sim.SimSleep(t,l,1.0)  #Wait for reset action is done
    l.exec_status= SUCCESS_CODE

  def InferPour():
    #Plan pour
    l.planlearn_callback(t,l,sim,'infer_pour')
    Log('After infer_pour')

  def PrePour():
    l.exec_status= PrePourSM(t,l,sim)
    sim.SimSleep(t,l,0.2)  #Wait for container movement when colliding
    l.planlearn_callback(t,l,sim,'end_of_prepour')
    Log('After end_of_prepour')

  def FlowCGen():
    l.flow_controlling= True
    l.exec_status= FlowCGenSM(t,l,sim)
    l.planlearn_callback(t,l,sim,'end_of_flowc_gen')
    l.flow_controlling= False
    Log('After end_of_flowc_gen')


  sm= TStateMachine()
  sm.EventCallback= lambda sm,et,st,ac: t.SMCallback(sm,et,st,ac)
  sm.Debug= True

  sm.StartState= 'start'

  sm.NewState('start')
  sm['start'].EntryAction= Setup
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'infer_grab'

  sm.NewState('infer_grab')
  sm['infer_grab'].EntryAction= InferGrab
  sm['infer_grab'].NewAction()
  sm['infer_grab'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['infer_grab'].Actions[-1].NextState= 'grab'
  sm['infer_grab'].ElseAction.Condition= lambda: True
  sm['infer_grab'].ElseAction.Action= sm.SetFailure
  sm['infer_grab'].ElseAction.NextState= EXIT_STATE

  sm.NewState('grab')
  sm['grab'].EntryAction= Grab
  sm['grab'].NewAction()
  sm['grab'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['grab'].Actions[-1].NextState= 'infer_pour'
  sm['grab'].ElseAction.Condition= lambda: True
  sm['grab'].ElseAction.Action= sm.SetFailure
  sm['grab'].ElseAction.NextState= EXIT_STATE

  sm.NewState('infer_pour')
  sm['infer_pour'].EntryAction= InferPour
  sm['infer_pour'].NewAction()
  sm['infer_pour'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['infer_pour'].Actions[-1].NextState= 'prepour'
  sm['infer_pour'].ElseAction.Condition= lambda: True
  sm['infer_pour'].ElseAction.Action= sm.SetFailure
  sm['infer_pour'].ElseAction.NextState= EXIT_STATE

  sm.NewState('prepour')
  sm['prepour'].EntryAction= PrePour
  sm['prepour'].NewAction()
  sm['prepour'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['prepour'].Actions[-1].NextState= 'flowc_gen'
  sm['prepour'].ElseAction.Condition= lambda: True
  sm['prepour'].ElseAction.Action= sm.SetFailure
  sm['prepour'].ElseAction.NextState= EXIT_STATE

  sm.NewState('flowc_gen')
  sm['flowc_gen'].EntryAction= FlowCGen
  sm['flowc_gen'].NewAction()
  sm['flowc_gen'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['flowc_gen'].Actions[-1].NextState= EXIT_STATE
  sm['flowc_gen'].ElseAction.Condition= lambda: True
  sm['flowc_gen'].ElseAction.Action= sm.SetFailure
  sm['flowc_gen'].ElseAction.NextState= EXIT_STATE

  try:
    t.RunSM(sm,'pour')

  except Exception as e:
    PrintException(e, ' in tsimh1.sm1')

  finally:
    sim.StopPubSub(t,l)
    l.sensor_callback= None
    t.srvp.ode_pause()

  return sm.ExitStatus


def TestPlanLearnCallback(t,l,sim,context):
  if context=='infer_grab':
    #Plan l.config.GripperHeight
    l.config.GripperHeight= 0.5*l.sensors.p_pour[2]  #Should be in [0,l.sensors.p_pour[2]]
    #l.config.GripperHeight= 0.8*l.sensors.p_pour[2]  #Should be in [0,l.sensors.p_pour[2]]
    l.exec_status= SUCCESS_CODE
  elif context=='infer_pour':
    #Plan l.p_pour_trg, l.p_pour_trg0, l.theta_init
    #l.p_pour_trg= [l.sensors.x_rcv.position.x-0.1, 0.0, l.sensors.x_rcv.position.z+0.4]
    #l.p_pour_trg0= Vec(l.p_pour_trg)+[0.0,0.0,0.2]
    #l.p_pour_trg= [l.sensors.x_rcv.position.x-0.15, 0.0, l.sensors.x_rcv.position.z+0.6]
    #l.p_pour_trg0= Vec(l.p_pour_trg)+[0.0,0.0,0.1]
    l.p_pour_trg= [l.sensors.x_rcv.position.x-0.1, 0.0, l.sensors.x_rcv.position.z+0.172]
    l.p_pour_trg0= Vec(l.p_pour_trg)+[0.0,0.0,0.2]
    #l.p_pour_trg= [l.sensors.x_rcv.position.x-0.4, 0.0, l.sensors.x_rcv.position.z+0.3]
    #l.p_pour_trg0= Vec(l.p_pour_trg)+[0.0,0.0,0.2]
    l.theta_init= DegToRad(45.0)
    l.exec_status= SUCCESS_CODE

def TestConfigCallback(t,l,sim):
  #l.config.RcvPos= [0.6, l.config.RcvPos[1], l.config.RcvPos[2]]
  l.config.RcvPos= [0.8, l.config.RcvPos[1], l.config.RcvPos[2]]
  #l.config.RcvPos= [1.2, l.config.RcvPos[1], l.config.RcvPos[2]]
  #l.config.RcvPos= [0.8+0.6*(random.random()-0.5), l.config.RcvPos[1], l.config.RcvPos[2]]
  CPrint(3,'l.config.RcvPos=',l.config.RcvPos)
  #l.config.ContactBounce= 0.7
  #l.config.ContactBounce= 0.1+0.8*(random.random())
  #CPrint(3,'l.config.ContactBounce=',l.config.ContactBounce)


def Run(t,*args):
  l= TContainer(debug=True)

  l.planlearn_callback= TestPlanLearnCallback
  l.config_callback= TestConfigCallback

  res= PourSM(t,l)
  #print 'l.filtered=\n',l.filtered
  print 'term_flow_center=', l.filtered.term_flow_center
  print 'term_flow_var=', l.filtered.term_flow_var
  print 'term_flow_max_dist=', l.filtered.term_flow_max_dist
  l= None
  return res

