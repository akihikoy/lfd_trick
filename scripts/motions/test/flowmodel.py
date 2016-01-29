#!/usr/bin/python
from core_tool import *
def Help():
  return '''Flow model (theta0 model where flow starts) construction test.
  Assumptions:
    We use ODE to simulate pouring
  Usage: test_flowmodel
    '''
def Run(t,*args):
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

  l.initialized_time= rospy.Time.now()
  l.timer_time= 0.0
  l.max_theta= 179.0/180.0*math.pi
  l.theta= 0.0
  l.time_step= 0.01
  l.filter_size= 30

  def Close():
    #l.flow_sensor.sub_callback_a= None
    l.flow_sensor.Deactivate()
    del l.flow_sensor  #FIXME
    #if not l.logfp.closed:
      #l.logfp.close()
      #CPrint(1,'Logged to',l.log_file_nameB)
    t.pub.theta.unregister()
    del t.pub.theta
  l.Close= Close

  def IsFlowObserved():
    if l.flow_sensor.IsFlowDetected():
      CPrint(3, '###Flow detected at (t,theta)=',rospy.Time.now()-l.initialized_time,l.theta)
      #time.sleep(0.5)
      return True
    return False
  l.IsFlowObserved= IsFlowObserved

  def ControlStep(dtheta):
    l.theta+= dtheta*0.2*l.time_step
    t.pub.theta.publish(l.theta)
    time.sleep(l.time_step)
  l.ControlStep= ControlStep

  #def Logger(msg):
    #l.ctrl_type= '-'
    #l.behavior_type= '-'
    #l.logfp.write('%f %f %f %f %f %f %f %f %s %s\n' % (
                  #(rospy.Time.now()-l.initialized_time).to_sec(),
                  #l.flow_sensor.Amount(),
                  #l.amount_trg,
                  #l.flow_sensor.FlowA(),
                  #l.flow_sensor.FilteredFlowA(l.filter_size),
                  #l.flow_trg,
                  #l.theta,
                  #l.max_theta,
                  #l.ctrl_type, l.behavior_type))
  #l.Logger= Logger
  #l.log_file_nameB= '/tmp/test_flowc_%s.dat'%TimeStr()
  #l.logfp= file(l.log_file_nameB,'w')
  #l.flow_sensor.sub_callback_a= l.Logger


  sm= TStateMachine()
  sm.EventCallback= t.SMCallback
  sm.Debug= True

  sm.StartState= 'start'
  sm.NewState('start')
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: True
  sm['start'].Actions[-1].NextState= 'std_pour'

  #Standard flow amount controller
  sm.NewState('std_pour')
  sm['std_pour'].ElseAction.Condition= lambda: True
  sm['std_pour'].ElseAction.NextState= 'find_flow_p' #'to_initial'

  sm.NewState('find_flow_p')
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1].Condition= lambda: l.IsFlowObserved()
  sm['find_flow_p'].Actions[-1].NextState= 'stop'
  sm['find_flow_p'].ElseAction.Condition= lambda: True
  sm['find_flow_p'].ElseAction.Action= lambda: l.ControlStep(0.5)
  sm['find_flow_p'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('stop')
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.Action= lambda: Print('End of pouring')
  sm['stop'].ElseAction.NextState= EXIT_STATE


  #sm.Show()
  sm.Run()
  l.Close()
  l= None

  return sm.ExitStatus
