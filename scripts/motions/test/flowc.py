#!/usr/bin/python
from core_tool import *
def Help():
  return '''Flow speed controller test.
  Assumptions:
    We use a real camera / ODE simulated sensor to observe the flow speed.
    Human operator will rotate the source container according to the direction displayed / simulation window.
  Usage: test_flowc [AMOUNT_TRG [, MAX_DURATION ]]
    AMOUNT_TRG: Target amount (default=0.4)
    MAX_DURATION: Maximum duration (default=50.0)
    '''
def Run(t,*args):
  amount_trg= args[0] if len(args)>0 else 0.4
  max_duration= args[1] if len(args)>1 else 100.0

  l= TContainer(debug=True)
  m_flow_sensor_rgb= t.LoadMotion('flow_sensor_rgb')
  l.flow_sensor= m_flow_sensor_rgb.TFlowSensorRGB(t)
  l.flow_sensor.flow_amount_factor= 0.0035  #NOTE: Good for ode_pour_sim
  l.flow_sensor.Activate()
  time.sleep(0.5)
  print l.flow_sensor.Amount()

  if not l.flow_sensor.ObservedA():
    print "Error: amount is not observed"
    return False

  t.pub.theta= rospy.Publisher("/ode_pour_sim/theta", std_msgs.msg.Float64)

  l.amount_trg= amount_trg
  l.max_duration= max_duration
  l.is_poured= False
  l.first_poured_time= -1.0
  l.initialized_time= rospy.Time.now()
  l.timer_time= 0.0
  l.max_theta= 179.0/180.0*math.pi
  l.theta= 0.0
  l.time_step= 0.01
  l.filter_size= 30

  #l.viz= TSimpleVisualizer(rospy.Duration(5.0), name_space='visualizer_test_flowc')
  #l.viz.viz_frame= t.robot.BaseFrame


  def Close():
    l.flow_sensor.sub_callback_a= None
    l.flow_sensor.Deactivate()
    del l.flow_sensor  #FIXME
    if not l.logfp.closed:
      l.logfp.close()
      CPrint(1,'Logged to',l.log_file_nameB)
    t.pub.theta.unregister()
    del t.pub.theta
  l.Close= Close

  def IsFlowObserved():
    if l.flow_sensor.IsFlowDetected():
      CPrint(3, '###Flow detected')
      #time.sleep(0.5)
      return True
    return False
  l.IsFlowObserved= IsFlowObserved

  def IsPoured():
    a= l.flow_sensor.Amount()
    if a >= l.amount_trg:
      print 'IsPoured:',a,type(a)
      print 'Poured! (',a,' / ',l.amount_trg,')'
      if not l.is_poured:
        l.is_poured= True
        l.first_poured_time= (rospy.Time.now() - l.initialized_time).to_sec()
        sm.Debug= False
        CPrint(1,'Log file:',l.log_file_nameB)
      return True
    return False
  l.IsPoured= IsPoured

  def IsTimeout():
    l.elapsed_time= (rospy.Time.now() - l.initialized_time).to_sec()
    if l.elapsed_time > l.max_duration:
      print '###TIMEOUT!### (',l.flow_sensor.Amount(),' / ',l.amount_trg,')'
      return True
    return False
  l.IsTimeout= IsTimeout

  def ChargeTimer(dt):
    l.elapsed_time= (rospy.Time.now() - l.initialized_time).to_sec()
    l.timer_time= l.elapsed_time+dt
  l.ChargeTimer= ChargeTimer

  def IsTimerTimeout():
    l.elapsed_time= (rospy.Time.now() - l.initialized_time).to_sec()
    return l.timer_time<=l.elapsed_time
  l.IsTimerTimeout= IsTimerTimeout

  def IsThetaEqTo(th,threshold_rate=0.04):
    threshold= threshold_rate*l.max_theta
    return abs(l.theta-th) < threshold
  l.IsThetaEqTo= IsThetaEqTo

  def ControlStep(dtheta):
    if not l.is_poured:
      if abs(dtheta) < 0.01:
        CPrint(4, '=====', l.theta,l.flow_sensor.Amount(), l.flow_sensor.FilteredFlowA(l.filter_size))
      elif dtheta>0.0:
        CPrint(4, '+++++', l.theta,l.flow_sensor.Amount(), l.flow_sensor.FilteredFlowA(l.filter_size))
      elif dtheta<0.0:
        CPrint(4, '-----', l.theta,l.flow_sensor.Amount(), l.flow_sensor.FilteredFlowA(l.filter_size))
    l.theta+= dtheta*0.2*l.time_step
    t.pub.theta.publish(l.theta)
    #x= [0.]*7
    #x[:3]= [0.,0.,0.5]
    #x[3:]= QFromAxisAngle([1.,0.,0.],(math.pi-l.max_theta)+l.theta)
    #l.viz.AddCube(x, scale=[0.5,0.5,1.0], alpha=0.6, rgb=l.viz.ICol(1), mid=0)
    time.sleep(l.time_step)
  l.ControlStep= ControlStep

  l.flow_trg= 0.08
  def ControlStepV(flow_trg):
    #ControlStep(dtheta)
    flow= l.flow_sensor.FlowA()
    flow_f= l.flow_sensor.FilteredFlowA(l.filter_size)
    if 'flow_f_prev' not in l:  l.flow_f_prev= flow_f
    d_flow_f= flow_f - l.flow_f_prev
    if flow_trg-flow < 0.3*flow_trg:  # 0.5
      kp= 10.0
      dtheta= kp*(flow_trg-flow)
    else:
      kp= 5.0
      kd= 25.0
      dtheta= kp*(flow_trg-flow_f) - kd*d_flow_f
    l.flow_f_prev= flow_f
    #dtheta= 5.0*(flow_trg-flow)**2
    #print flow_trg-flow, kp, dtheta
    ControlStep(dtheta)
  l.ControlStepV= ControlStepV

  def Logger(msg):
    l.ctrl_type= '-'
    l.behavior_type= '-'
    l.logfp.write('%f %f %f %f %f %f %f %f %s %s\n' % (
                  (rospy.Time.now()-l.initialized_time).to_sec(),
                  l.flow_sensor.Amount(),
                  l.amount_trg,
                  l.flow_sensor.FlowA(),
                  l.flow_sensor.FilteredFlowA(l.filter_size),
                  l.flow_trg,
                  l.theta,
                  l.max_theta,
                  l.ctrl_type, l.behavior_type))
  l.Logger= Logger
  l.log_file_nameB= '/tmp/test_flowc_%s.dat'%TimeStr()
  l.logfp= file(l.log_file_nameB,'w')
  l.flow_sensor.sub_callback_a= l.Logger

  #CPrint(4, 'START AFTER 2 SEC')
  #time.sleep(2)

  sm= TStateMachine()
  sm.EventCallback= t.SMCallback
  sm.Debug= True

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= l.IsTimeout
  timeout_action.NextState= 'stop'

  poured_action= TFSMConditionedAction()
  poured_action.Condition= l.IsPoured
  poured_action.NextState= 'start'
  #poured_action.NextState= 'stop'

  sm.StartState= 'start'
  sm.NewState('start')
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= l.IsPoured
  sm['start'].Actions[-1].NextState= 'stop'
  sm['start'].NewAction()
  sm['start'].Actions[-1]= timeout_action
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: True
  sm['start'].Actions[-1].NextState= 'std_pour'

  #Standard flow amount controller
  sm.NewState('std_pour')
  sm['std_pour'].ElseAction.Condition= lambda: True
  sm['std_pour'].ElseAction.NextState= 'find_flow_p' #'to_initial'

  sm.NewState('find_flow_p')
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= poured_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= timeout_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1].Condition= lambda: l.IsFlowObserved()
  sm['find_flow_p'].Actions[-1].NextState= 'pour'
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1].Condition= lambda: l.IsThetaEqTo(l.max_theta)
  sm['find_flow_p'].Actions[-1].NextState= 'stop'  # 'start'
  sm['find_flow_p'].ElseAction.Condition= lambda: True
  sm['find_flow_p'].ElseAction.Action= lambda: l.ControlStep(0.5)
  sm['find_flow_p'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('pour')
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= poured_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= timeout_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= lambda: l.IsThetaEqTo(l.max_theta)
  sm['pour'].Actions[-1].NextState= 'start'
  sm['pour'].ElseAction.Condition= lambda: True
  sm['pour'].ElseAction.Action= lambda: l.ControlStepV(l.flow_trg)
  sm['pour'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('stop')
  sm['stop'].NewAction()
  sm['stop'].Actions[-1].Condition= lambda: l.IsThetaEqTo(0.0)
  sm['stop'].Actions[-1].Action= lambda: ( Print('End of pouring'), Print('First pour: ',l.first_poured_time) )
  sm['stop'].Actions[-1].NextState= EXIT_STATE
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.Action= lambda: l.ControlStep(-1.0)
  sm['stop'].ElseAction.NextState= ORIGIN_STATE


  #sm.Show()
  sm.Run()
  l.Close()
  l= None

  return sm.ExitStatus
