#!/usr/bin/python
from core_tool import *
def Help():
  return '''Common routines for flow amount controllers (ver.3).  Do not use this directly.'''
def Run(t,*args):
  print 'Error:',Help()

class TLocal(TContainer):
  def __init__(self,debug=False):
    TContainer.__init__(self,debug)

  def __del__(self):
    l= self; t= self.t
    l.Close()
    TContainer.__del__(self)

  def Setup(self, t, bottle, cup, amount_trg, max_duration, flow_trg):
    l= self
    self.t= t

    l.bottle= bottle
    l.cup= cup
    l.amount_trg= amount_trg
    l.max_duration= max_duration
    l.flow_trg= flow_trg

    if not t.HasAttr(l.bottle,'grabbed'):
      CPrint(4,'Error: not grabbed: ',l.bottle)
      return False

    l.using_cvfilter= True
    l.camera_id= 0  #Effective only if l.using_cvfilter; 0: both, 1: first cam, 2: second cam
    t.m.flow_sensor_rgb= t.LoadMotion('flow_sensor_rgb')
    if not l.using_cvfilter:
      #l.flow_sensor= t.m.flow_sensor_rgb.TFlowSensorRGB(t)
      l.flow_sensor= t.m.flow_sensor_rgb.TFlowAmountModel(t)  #NEW_TFlowAmountModel
    else:
      l.flow_sensor= t.m.flow_sensor_rgb.TFlowAmountModel(t=None)  #NEW_TFlowAmountModel
    if not l.flow_sensor.Activate():  #cv setup will be done if not l.using_cvfilter
      return False
    l.cv_ready= False
    if l.using_cvfilter:
      if not t.ExecuteMotion('cv', 'setup', l.camera_id):
        CPrint(4,'cv setup failed.')
        return False
      l.cv_ready= True
      t.m.cvfilter1= t.LoadMotion('cvfilter1')
      l.cvfiltered= TContainer(debug=True)
      t.ExecuteMotion('cv', 'resume', l.camera_id)
      t.callback.cv= lambda msg: (
          t.m.cvfilter1.CVApplyFilter(l.cvfiltered, msg, 1),
          t.m.cvfilter1.VizCVFiltered(t, l.cvfiltered, 1),
          t.m.cvfilter1.PassToOldCallbacks(msg, l.cvfiltered, 1,
                                           amount_observer=l.flow_sensor.CallbackA,
                                           flow_speed_observer=l.flow_sensor.CallbackF) )
      t.callback.cv2= lambda msg: (
          t.m.cvfilter1.CVApplyFilter(l.cvfiltered, msg, 2),
          t.m.cvfilter1.VizCVFiltered(t, l.cvfiltered, 2) )
    time.sleep(1.0)  #Wait for sensor observation
    print l.flow_sensor.Amount()
    for i in range(100):
      if l.flow_sensor.amount < l.amount_trg:
        break
      print l.flow_sensor.Amount()
      time.sleep(0.05)

    if not l.flow_sensor.observed_a:
      CPrint(4,'Error: amount is not observed')
      return False

    l.i= t.GetAttr(l.bottle,'grabbed','grabber_handid')


    #>>>FIXME: The same computation of lw_x_pour_e is used in m_pour.py
    #Infer l.bottle pose
    t.ExecuteMotion('infer',  l.bottle,'x')
    x_b= t.GetAttr(l.bottle,'x')
    if len(x_b)!=7:
      CPrint(4,'Bottle ',l.bottle,' pose is not observed')
      return False
    #Pouring edge point on the l.bottle frame:
    lb_x_pour_e= t.GetAttr(l.bottle,'l_x_pour_e')
    x_w= t.robot.FK(arm=l.i)
    #Pouring edge point in the wrist frame
    lw_x_pour_e= TransformLeftInv(x_w, Transform(x_b,lb_x_pour_e))
    print 'lw_x_pour_e=',VecToStr(lw_x_pour_e)
    #<<<FIXME


    #Estimate axis and max-angle for flow amount control(ver.1)
    #xe_init= t.robot.FK(x_ext=lw_x_pour_e, arm=l.i)
    #p_init, R_init= XToPosRot(xe_init)
    #R_max= QToRot(t.GetAttr(l.bottle,'q_pour_max'))
    #trans_R= np.dot(R_max,R_init.T)
    #axis_angle= InvRodrigues(trans_R)
    #max_theta= la.norm(axis_angle)
    #axis= axis_angle / max_theta

    #Estimate axis and max-angle for flow amount control(ver.2)
    p_b,R_b= XToPosRot(x_b)
    print 'R_b:',R_b
    print 'la.norm(R_b[:,2]):',la.norm(R_b[:,2])
    ax_gravity= [0,0,-1]  #TODO: define in attributes
    axis= np.cross(R_b[:,2],ax_gravity)
    axis= axis / la.norm(axis)
    max_theta= math.acos(np.dot(R_b[:,2],ax_gravity))

    #<<<FIXME: the above code is copied from m_flowc.py; a part of code is overwrapping with the below code!!!

    print 'axis: %r (%f)' % (axis, la.norm(axis))
    print 'max_theta:',max_theta
    #if not t.AskYesNo():  return False

    #Control constants:
    l.flow_control_dtheta_max= math.pi*0.1
    l.flow_control_dtheta_min= -math.pi*0.1
    #l.flow_control_time_step= 0.01
    l.flow_control_time_step= 0.02  #New_ctrl_from_vernon_20150913
    if not 'warning_flow_control_time_step_change' in t.__dict__:
      CPrint(4,'flowc3_cmn: flow_control_time_step changed ',
             'since we are controlling from laptop @2015-09-13.',
             'OK?  Y:continue, N:raise exception.')
      time.sleep(0.5)
      #if not t.AskYesNo():
        #raise Exception('flowc3_cmn: Check the value of flow_control_time_step.')
      t.warning_flow_control_time_step_change= 'User approved.'

    l.shake_widthB_ext= None  #externally planned shaking B width

    l.rot_axis= axis
    l.max_theta= max_theta
    l.x_ext= lw_x_pour_e

    #angles_init= t.robot.Q(l.i)
    l.x_init= np.array(t.robot.FK(x_ext=l.x_ext, arm=l.i))
    l.x_init2= copy.deepcopy(l.x_init)

    l.theta= 0.0
    l.elapsed_time= 0.0
    l.timer_time= 0.0

    l.m_infer= t.LoadMotion('infer')

    l.initialized_time= rospy.Time.now()
    l.is_poured= False
    l.is_spreaded= False
    l.first_poured_time= -1.0

    time_str= TimeStr('short2')
    l.get_local_tmp= lambda s: t.LogFileName(s,time_str)
    l.log_file_nameB= l.get_local_tmp('flowcB')
    l.ctrl_type= '-'
    l.behavior_type= '-'
    print 'Logging to',l.log_file_nameB
    l.logfp= file(l.log_file_nameB,'w')
    l.flow_sensor.sub_callback_a= l.Logger
    #time.sleep(10.0)

    return True

  def Close(self):
    l= self; t= self.t
    t.callback.cv= None
    t.callback.cv2= None
    l.cvfiltered= None
    l.flow_sensor.sub_callback_a= None
    l.flow_sensor.Deactivate()
    if l.using_cvfilter and l.cv_ready:
      t.ExecuteMotion('cv', 'pause', l.camera_id)
    if 'logfp' in l and not l.logfp.closed:
      l.logfp.close()
      CPrint(3,'Logged to',l.log_file_nameB)
    #WriteVarToFile(l.flow_sensor, l.get_local_tmp('flowc_fs_'))  #FIXME
    #CPrint(3, 'Saved l.flow_sensor to ',l.get_local_tmp('flowc_fs_'))

  def Reset(self):
    l= self; t= self.t
    l.Close()
    l.Setup(t, l.bottle, l.amount_trg, l.max_duration, l.flow_trg)

  def Logger(self, msg):
    l= self; t= self.t
    #l.logfp.write('%f %f %f %f %f %s %s\n' % (
                  #rospy.Time.now().to_nsec(),
                  #l.flow_sensor.amount,
                  #l.amount_trg,
                  #l.theta,
                  #l.max_theta,
                  #l.ctrl_type,
                  #l.behavior_type))
    l.logfp.write('%f %f %f %f %f %f %f %f %f %f %f %f %s %s\n' % (
                  rospy.Time.now().to_nsec(),
                  l.flow_sensor.Amount(),
                  l.amount_trg,
                  l.flow_sensor.AmountSrc(),  #NEW_TFlowAmountModel
                  l.flow_sensor.FlowA(),
                  l.flow_sensor.FilteredFlowA(filter_size=30),
                  l.flow_trg,
                  l.flow_sensor.flow_amount_factor,
                  l.theta,  #relative value from initial angle
                  l.max_theta,
                  l.theta+(math.pi-l.max_theta),  #absolute value
                  l.flow_sensor.Theta0(),  #NEW_TFlowAmountModel
                  l.ctrl_type, l.behavior_type))

  def IsFlowObserved(self):
    l= self; t= self.t
    if l.flow_sensor.IsFlowDetected():
      CPrint(3, '###Flow detected')
      #time.sleep(0.5)
      return True
    return False

  def IsPoured(self):
    l= self; t= self.t
    if l.is_spreaded:  return True  #Hack to stop SMs for pouring when spreading is finished
    a= l.flow_sensor.amount
    if a >= l.amount_trg:
      print 'Poured! (',a,' / ',l.amount_trg,')'
      if not l.is_poured:
        l.is_poured= True
        l.first_poured_time= (rospy.Time.now() - l.initialized_time).to_sec()
      return True
    return False

  def IsTimeout(self):
    l= self; t= self.t
    if l.elapsed_time > l.max_duration:
      print '###TIMEOUT!### (',l.flow_sensor.amount,' / ',l.amount_trg,')'
      return True
    return False

  def ChargeTimer(self,dt):
    l= self; t= self.t
    l.timer_time= l.elapsed_time+dt

  def IsTimerTimeout(self):
    l= self; t= self.t
    return l.timer_time<=l.elapsed_time

  def IsThetaEqTo(self,th,threshold_rate=0.001):
    l= self; t= self.t
    threshold= threshold_rate*l.max_theta
    return abs(l.theta-th) < threshold

  def ControlStep(self,dtheta):
    l= self; t= self.t
    l.ctrl_type= 'c'

    if dtheta > l.flow_control_dtheta_max:  dtheta= l.flow_control_dtheta_max
    elif dtheta < l.flow_control_dtheta_min:  dtheta= l.flow_control_dtheta_min
    l.theta= l.theta+dtheta * l.flow_control_time_step
    if l.theta > l.max_theta:  l.theta= l.max_theta
    elif l.theta < 0.0:  l.theta= 0.0

    #print l.elapsed_time,': ',l.flow_sensor.amount,' / ',l.amount_trg,' : ',l.theta,', ',dtheta
    print '%f : %f / %f (%f, %f) : %f , %f \n' % (
        l.elapsed_time,
        l.flow_sensor.Amount(),
        l.amount_trg,
        l.flow_sensor.FilteredFlowA(filter_size=30),
        l.flow_sensor.flow_amount_factor,
        l.theta,
        dtheta
      )

    p_init,R_init= XToPosRot(l.x_init)
    dR= QToRot(QFromAxisAngle(l.rot_axis,l.theta))
    R_trg= np.dot(dR,R_init)
    x_trg= PosRotToX(p_init,R_trg)
    #print '##',VecToStr(x_trg)

    #Store x_trg in future use (cf. Shake)
    l.x_init2= copy.deepcopy(x_trg)

    angles_curr= t.robot.Q(l.i)
    angles,res= t.robot.IK(x_trg, l.x_ext, angles_curr, arm=l.i, with_st=True)
    if angles==None:
      raise Exception("IK error: %r"%res.error_code.val)

    t.robot.FollowQTraj([angles], [l.flow_control_time_step], arm=l.i)
    start_time= rospy.Time.now()
    while rospy.Time.now() < start_time + rospy.Duration(l.flow_control_time_step):
      time.sleep(l.flow_control_time_step*0.02)

    l.elapsed_time+= l.flow_control_time_step

  #Velocity controller
  def ControlStepV(self,flow_trg):
    l= self; t= self.t
    flow= l.flow_sensor.FlowA()
    flow_f= l.flow_sensor.FilteredFlowA(filter_size=30)  #FIXME:magic number
    if 'flow_f_prev' not in l.__dict__:  l.flow_f_prev= flow_f
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
    l.ControlStep(dtheta)

  def Shake(self,count,lb_axis_shake,shake_width,shake_freq):
    l= self; t= self.t
    l.ctrl_type= 's2'

    print 'shake:',l.elapsed_time,': ',l.flow_sensor.amount,' / ',l.amount_trg,' : ',shake_freq

    dt= 1.0/shake_freq

    #>>>Shaking motion
    x_trg1= copy.deepcopy(l.x_init2)
    x_trg2= copy.deepcopy(l.x_init2)
    l.m_infer.Run(t, l.bottle,'x')
    x_b= t.GetAttr(l.bottle,'x')  #Bottle pose in robot frame
    p_b,R_b= XToPosRot(x_b)
    axis_shake= np.dot(R_b,lb_axis_shake)  #Shaking axis in robot frame
    axis_shake= np.array(axis_shake) / la.norm(axis_shake)
    x_trg1[0:3]+= np.array(axis_shake)*(0.5*shake_width)
    x_trg2[0:3]-= np.array(axis_shake)*(0.5*shake_width)

    '''
    #inum= 5
    inum= 10
    t.robot.MoveToXI(x_trg1,dt/2.0,l.x_ext,inum, arm=l.i,blocking='time')
    l.elapsed_time+= dt/2.0
    for n in range(count-1):
      t.robot.MoveToXI(x_trg2,dt/2.0,l.x_ext,inum, arm=l.i,blocking='time')
      t.robot.MoveToXI(x_trg1,dt/2.0,l.x_ext,inum, arm=l.i,blocking='time')
      l.elapsed_time+= dt
    t.robot.MoveToXI(x_trg2,dt/2.0,l.x_ext,inum, arm=l.i,blocking='time')
    l.elapsed_time+= dt/2.0
    t.robot.MoveToXI(l.x_init2,dt/2.0,l.x_ext,inum, arm=l.i,blocking='time')
    l.elapsed_time+= dt/2.0
    '''
    #New_shake_2015_09_11
    x_traj= []; t_traj= []
    add_traj= lambda x,dt: (x_traj.append(x), t_traj.append(dt+(t_traj[-1] if len(t_traj)>0 else 0.0)))
    add_traj(x_trg1, dt/2.0)
    for n in range(count-1):
      add_traj(x_trg2, dt/2.0)
      add_traj(x_trg1, dt/2.0)
    add_traj(x_trg2, dt/2.0)
    add_traj(l.x_init2, dt/2.0)
    t.robot.FollowXTraj(x_traj, t_traj, x_ext=l.x_ext, arm=l.i,blocking='time')
    l.elapsed_time+= t_traj[-1]
    #'''
    #<<<Shaking motion


#Standard flow amount controller
def GenSMStdPour(t,local_obj):
  sm= TStateMachine(debug=True, local_obj=local_obj)
  sm.EventCallback= t.SMCallback

  poured_action= TFSMConditionedAction()
  poured_action.Condition= sm.l.IsPoured
  poured_action.NextState= EXIT_STATE

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= EXIT_STATE

  sm.StartState= 'to_initial'

  sm.NewState('to_initial')
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1]= poured_action
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1]= timeout_action
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1].Condition= lambda: sm.l.IsThetaEqTo(0.0)
  #sm['to_initial'].Actions[-1].NextState= 'find_flow_p'
  sm['to_initial'].Actions[-1].NextState= 'approach'  #NEW_TFlowAmountModel
  sm['to_initial'].ElseAction.Condition= lambda: True
  sm['to_initial'].ElseAction.Action= lambda: sm.l.ControlStep(sm.l.flow_control_dtheta_min)
  sm['to_initial'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('approach')  #NEW_TFlowAmountModel
  sm['approach'].NewAction()
  sm['approach'].Actions[-1]= poured_action
  sm['approach'].NewAction()
  sm['approach'].Actions[-1]= timeout_action
  sm['approach'].NewAction()
  sm['approach'].NewAction()
  sm['approach'].Actions[-1].Condition= lambda: sm.l.theta+(math.pi-sm.l.max_theta)>sm.l.flow_sensor.Theta0()-0.1
  sm['approach'].Actions[-1].NextState= 'find_flow_p'
  sm['approach'].ElseAction.Condition= lambda: True
  sm['approach'].ElseAction.Action= lambda: sm.l.ControlStep(0.7 * sm.l.flow_control_dtheta_max)
  sm['approach'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('find_flow_p')
  sm['find_flow_p'].EntryAction= lambda: sm.l.IsFlowObserved()  #Ignore previous cache
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= poured_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= timeout_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1].Condition= lambda: sm.l.IsFlowObserved()
  sm['find_flow_p'].Actions[-1].Action= lambda: sm.l.flow_sensor.UpdateAtInitialFlow(sm.l.theta)  #NEW_TFlowAmountModel
  sm['find_flow_p'].Actions[-1].NextState= 'pour'
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1].Condition= lambda: sm.l.IsThetaEqTo(sm.l.max_theta)
  sm['find_flow_p'].Actions[-1].NextState= EXIT_STATE
  sm['find_flow_p'].ElseAction.Condition= lambda: True
  sm['find_flow_p'].ElseAction.Action= lambda: sm.l.ControlStep(0.2 * sm.l.flow_control_dtheta_max)  #FIXME: magic number
  sm['find_flow_p'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('pour')
  sm['pour'].EntryAction= lambda: sm.l.ChargeTimer(5.0)  #Time of patience
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= poured_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= timeout_action
  sm['pour'].NewAction()
  #sm['pour'].Actions[-1].Condition= lambda: sm.l.IsFlowObserved()
  sm['pour'].Actions[-1].Condition= lambda: sm.l.flow_sensor.FilteredFlowA(filter_size=10)>0.001
  sm['pour'].Actions[-1].Action= lambda: ( sm.l.ChargeTimer(5.0), sm.l.ControlStep(0.0) )
  sm['pour'].Actions[-1].NextState= ORIGIN_STATE
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= lambda: sm.l.IsTimerTimeout()
  sm['pour'].Actions[-1].NextState= EXIT_STATE
  sm['pour'].ElseAction.Condition= lambda: True
  sm['pour'].ElseAction.Action= lambda: sm.l.ControlStep(0.2 * sm.l.flow_control_dtheta_max)  #FIXME: magic number
  sm['pour'].ElseAction.NextState= ORIGIN_STATE
  #sm.NewState('pour')
  #sm['pour'].NewAction()
  #sm['pour'].Actions[-1]= poured_action
  #sm['pour'].NewAction()
  #sm['pour'].Actions[-1]= timeout_action
  #sm['pour'].NewAction()
  #sm['pour'].Actions[-1].Condition= lambda: sm.l.IsThetaEqTo(sm.l.max_theta)
  #sm['pour'].Actions[-1].NextState= EXIT_STATE
  #sm['pour'].ElseAction.Condition= lambda: True
  #sm['pour'].ElseAction.Action= lambda: sm.l.ControlStepV(sm.l.flow_trg)
  #sm['pour'].ElseAction.NextState= ORIGIN_STATE

  return sm


#Shake-A flow amount controller
def GenSMShakeA(t,local_obj):
  sm= TStateMachine(debug=True, local_obj=local_obj)
  sm.EventCallback= t.SMCallback

  #sm.l.shake_freq= 2.0
  sm.l.shake_freq= 2.5
  #sm.l.shake_freq= 3.0
  #sm.l.shake_widthA= 0.06
  sm.l.shake_widthA= 0.08  #New_shake_2015_09_11
  sm.l.lb_axis_shake= [0.0,0.0,-1.0]
  #sm.l.lb_axis_shake= np.array(sm.l.lb_axis_shake)/la.norm(sm.l.lb_axis_shake)

  poured_action= TFSMConditionedAction()
  poured_action.Condition= sm.l.IsPoured
  poured_action.NextState= EXIT_STATE

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= EXIT_STATE

  sm.StartState= 'to_initial'

  sm.NewState('to_initial')
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1]= poured_action
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1]= timeout_action
  sm['to_initial'].NewAction()
  sm['to_initial'].Actions[-1].Condition= lambda: sm.l.IsThetaEqTo(0.0)
  sm['to_initial'].Actions[-1].NextState= 'to_max'
  sm['to_initial'].ElseAction.Condition= lambda: True
  sm['to_initial'].ElseAction.Action= lambda: sm.l.ControlStep(sm.l.flow_control_dtheta_min)
  sm['to_initial'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('to_max')
  sm['to_max'].NewAction()
  sm['to_max'].Actions[-1]= poured_action
  sm['to_max'].NewAction()
  sm['to_max'].Actions[-1]= timeout_action
  sm['to_max'].NewAction()
  sm['to_max'].Actions[-1].Condition= lambda: sm.l.IsThetaEqTo(sm.l.max_theta)
  sm['to_max'].Actions[-1].NextState= 'shake'
  sm['to_max'].ElseAction.Condition= lambda: True
  sm['to_max'].ElseAction.Action= lambda: sm.l.ControlStep(sm.l.flow_control_dtheta_max)
  sm['to_max'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('shake')
  sm['shake'].EntryAction= lambda: sm.l.ChargeTimer(3.0)
  sm['shake'].NewAction()
  sm['shake'].Actions[-1]= poured_action
  sm['shake'].NewAction()
  sm['shake'].Actions[-1]= timeout_action
  sm['shake'].NewAction()
  sm['shake'].Actions[-1].Condition= lambda: sm.l.IsFlowObserved()
  sm['shake'].Actions[-1].Action= lambda: ( sm.l.ChargeTimer(3.0), sm.l.Shake(2, sm.l.lb_axis_shake, sm.l.shake_widthA, sm.l.shake_freq) )
  sm['shake'].Actions[-1].NextState= ORIGIN_STATE
  sm['shake'].NewAction()
  sm['shake'].Actions[-1].Condition= lambda: not sm.l.IsTimerTimeout()
  sm['shake'].Actions[-1].Action= lambda: ( sm.l.Shake(2, sm.l.lb_axis_shake, sm.l.shake_widthA, sm.l.shake_freq) )
  sm['shake'].Actions[-1].NextState= ORIGIN_STATE
  sm['shake'].ElseAction.Condition= lambda: True
  sm['shake'].ElseAction.NextState= EXIT_STATE

  return sm


#Shake-B flow amount controller
def GenSMShakeB(t,local_obj):
  sm= TStateMachine(debug=True, local_obj=local_obj)
  sm.EventCallback= t.SMCallback

  if t.robot.Is('PR2'):
    #sm.l.shake_freq= 2.0
    sm.l.shake_freq= 2.5
    #sm.l.shake_freq= 3.0
    #sm.l.shake_width= 0.025
    sm.l.shake_width= 0.032  #New_shake_2015_09_11
  elif t.robot.Is('Baxter'):
    sm.l.shake_freq= 2.5
    sm.l.shake_width= 0.06
    #t.SetAttr('memory','flowc','b55','shake_axis_theta_means', [math.pi/3.0])  #TEST
  if sm.l.shake_widthB_ext!=None:  sm.l.shake_width= sm.l.shake_widthB_ext  #externally planned
  #sm.l.shake_width= 0.03

  poured_action= TFSMConditionedAction()
  poured_action.Condition= sm.l.IsPoured
  poured_action.NextState= EXIT_STATE

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= EXIT_STATE


  #Search the best lb_axis_shake
  sm.Params['shake_axis_theta']= TContParamNoGrad()
  shake_axis_theta= sm.Params['shake_axis_theta']
  shake_axis_theta.Mean= t.GetAttrOr([math.pi/4.0], 'memory','flowc',sm.l.bottle,'shake_axis_theta_means')
  shake_axis_theta.Std= t.GetAttrOr(1.0, 'memory','flowc',sm.l.bottle,'shake_axis_theta_std')
  shake_axis_theta.Min= [0.0]
  shake_axis_theta.Max= [math.pi/2.0]
  shake_axis_theta.Init()  #FIXME:Use d=Save() and Init(d) to save and restore the data
  sm.l.shake_axis_can_be_updated= False
  def select_shake_axis():
    sm.l.shake_axis_amount_begin= sm.l.flow_sensor.amount
    sm.l.shake_axis_time_begin= sm.l.elapsed_time
    shake_axis_theta.Select()
    sm.l.shake_axis_can_be_updated= True
  def get_shake_axis():
    if not sm.l.updating_param_c:
      #th= 0.0
      th= math.pi/4.0
    else:
      th= shake_axis_theta.Param()[0]
    return [math.sin(th),0.0,-math.cos(th)]
  def update_shake_axis():
    if sm.l.updating_param_c and sm.l.shake_axis_can_be_updated:
      score= 100.0*(sm.l.flow_sensor.amount - sm.l.shake_axis_amount_begin) / (sm.l.elapsed_time - sm.l.shake_axis_time_begin)
      shake_axis_theta.Update(score)
      sm.l.shake_axis_can_be_updated= False
      print '###DEBUG:',sm.l.shake_axis_amount_begin,sm.l.flow_sensor.amount, '@',sm.l.elapsed_time - sm.l.shake_axis_time_begin
      #print '###DEBUG'; t.AskYesNo()


  sm.StartState= 'find_flow_m'

  sm.NewState('find_flow_m')
  sm['find_flow_m'].EntryAction= lambda: sm.l.IsFlowObserved()  #Ignore previous cache
  sm['find_flow_m'].NewAction()
  sm['find_flow_m'].Actions[-1]= poured_action
  sm['find_flow_m'].NewAction()
  sm['find_flow_m'].Actions[-1]= timeout_action
  sm['find_flow_m'].NewAction()
  sm['find_flow_m'].Actions[-1].Condition= lambda: sm.l.IsFlowObserved()
  sm['find_flow_m'].Actions[-1].NextState= 'shake2'
  sm['find_flow_m'].NewAction()
  sm['find_flow_m'].Actions[-1].Condition= lambda: sm.l.IsThetaEqTo(0.0)
  sm['find_flow_m'].Actions[-1].NextState= 'find_flow_p'
  sm['find_flow_m'].ElseAction.Condition= lambda: True
  sm['find_flow_m'].ElseAction.Action= lambda: sm.l.ControlStep(0.5 * sm.l.flow_control_dtheta_min)
  sm['find_flow_m'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('find_flow_p')
  sm['find_flow_p'].EntryAction= lambda: sm.l.IsFlowObserved()  #Ignore previous cache
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= poured_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1]= timeout_action
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1].Condition= lambda: sm.l.IsFlowObserved()
  sm['find_flow_p'].Actions[-1].NextState= 'shake2'
  sm['find_flow_p'].NewAction()
  sm['find_flow_p'].Actions[-1].Condition= lambda: sm.l.IsThetaEqTo(sm.l.max_theta)
  sm['find_flow_p'].Actions[-1].NextState= 'shake2'
  sm['find_flow_p'].ElseAction.Condition= lambda: True
  sm['find_flow_p'].ElseAction.Action= lambda: sm.l.ControlStep(0.5 * sm.l.flow_control_dtheta_max)
  sm['find_flow_p'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('shake2')
  shake2_action= lambda: ( select_shake_axis(), sm.l.Shake(2, get_shake_axis(), sm.l.shake_width, sm.l.shake_freq), update_shake_axis() )
  sm['shake2'].EntryAction= lambda: sm.l.ChargeTimer(3.0)
  sm['shake2'].NewAction()
  sm['shake2'].Actions[-1]= poured_action
  sm['shake2'].NewAction()
  sm['shake2'].Actions[-1]= timeout_action
  sm['shake2'].NewAction()
  sm['shake2'].Actions[-1].Condition= lambda: sm.l.IsFlowObserved()
  sm['shake2'].Actions[-1].Action= lambda: ( sm.l.ChargeTimer(3.0), shake2_action() )
  sm['shake2'].Actions[-1].NextState= ORIGIN_STATE
  sm['shake2'].NewAction()
  sm['shake2'].Actions[-1].Condition= lambda: not sm.l.IsTimerTimeout()
  sm['shake2'].Actions[-1].Action= lambda: shake2_action()
  sm['shake2'].Actions[-1].NextState= ORIGIN_STATE
  sm['shake2'].ElseAction.Condition= lambda: True
  sm['shake2'].ElseAction.NextState= EXIT_STATE

  return sm


#Control left arm downward to make the pouring edge (source) close to the pouring location (receiver)
def GenSMPourHeightCtrl(t,local_obj):
  sm= TStateMachine(debug=False, local_obj=local_obj)
  #sm.EventCallback= t.SMCallback  #WARNING: No callback to avoide confuse the context

  poured_action= TFSMConditionedAction()
  poured_action.Condition= sm.l.IsPoured
  poured_action.NextState= 'stop'

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= 'stop'

  sm.StartState= 'init'

  def Init():
    t.ExecuteMotion('infer', sm.l.cup,'x')
    x_c= t.GetAttr(sm.l.cup,'x')
    sm.l.z_max= Transform(x_c, t.GetAttr(sm.l.cup,'l_x_pour_l0'))[2]
    sm.l.z_min= Transform(x_c, t.GetAttr(sm.l.cup,'l_x_pour_l'))[2]
    sm.l.x_init_z= sm.l.x_init[2]
    if sm.l.z_max<sm.l.x_init_z:  sm.l.z_max= sm.l.x_init_z
    if sm.l.z_min>sm.l.x_init_z:  sm.l.z_min= sm.l.x_init_z
    t.ExecuteMotion('scene', 'make')

  def HeadCtrlStep():
    dt= sm.l.flow_control_time_step
    z_step= 0.02*dt

    p_init,R_init= XToPosRot(sm.l.x_init)
    dR= QToRot(QFromAxisAngle(sm.l.rot_axis,sm.l.theta))
    R_curr= np.dot(dR,R_init)
    x_curr= PosRotToX(p_init,R_curr)

    x_trg= TransformRightInv(x_curr, sm.l.x_ext)
    if not (t.ExecuteMotion('scene','isvalidx',sm.l.i,x_trg,[],t.robot.Q(sm.l.i),True))[0]:
      #If collision
      if sm.l.x_init[2]<sm.l.z_max:
        sm.l.x_init[2]+= z_step
        if sm.l.x_init[2]>sm.l.z_max:
          sm.l.x_init[2]= sm.l.z_max
        print 'Move z to',sm.l.x_init[2]
    else:
      if sm.l.x_init[2]>sm.l.z_min:
        x_curr[2]-= z_step
        if x_curr[2]<sm.l.z_min:
          x_curr[2]= sm.l.z_min
      x_trg= TransformRightInv(x_curr, sm.l.x_ext)
      if (t.ExecuteMotion('scene','isvalidx',sm.l.i,x_trg,[],t.robot.Q(sm.l.i),True))[0]:
        #Can move to x_curr[2]
        sm.l.x_init[2]= x_curr[2]
        print 'Move z to',sm.l.x_init[2]
    time.sleep(dt)

  def StopStep():
    dt= sm.l.flow_control_time_step
    z_step= 0.02*dt
    x_trg= TransformRightInv(sm.l.x_init, sm.l.x_ext)
    if not (t.ExecuteMotion('scene','isvalidx',sm.l.i,x_trg,[],t.robot.Q(sm.l.i),True))[0]:
      #If collision
      if sm.l.x_init[2]<sm.l.z_max:
        sm.l.x_init[2]+= z_step
        print 'Move z to',sm.l.x_init[2]
        time.sleep(dt)


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
  sm['stop'].ElseAction.Action= lambda: t.ExecuteMotion('scene', 'clear')
  sm['stop'].ElseAction.NextState= EXIT_STATE

  return sm


#Control left arm for spreading (line movement)
def GenSMSpreadCtrl1(t,local_obj):
  sm= TStateMachine(debug=False, local_obj=local_obj)
  #sm.EventCallback= t.SMCallback  #WARNING: No callback to avoide confuse the context

  poured_action= TFSMConditionedAction()
  poured_action.Condition= sm.l.IsPoured
  poured_action.NextState= EXIT_STATE

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= EXIT_STATE

  sm.StartState= 'init'

  def Init():
    sm.l.x_init3= copy.deepcopy(sm.l.x_init)
    sm.l.diff_y= 0.0

  def SpreadCtrlStep():
    #diff= [0.02*math.sin(4.0*sm.l.elapsed_time),-0.05*(math.cos(1.0*sm.l.elapsed_time)-1.0)]
    vel_y= 0.5*sm.l.flow_sensor.FilteredFlowA(filter_size=5)
    if vel_y>0.03:  vel_y= 0.03
    if vel_y<0.0:   vel_y= 0.0
    sm.l.diff_y+= sm.l.flow_control_time_step*vel_y
    if sm.l.diff_y>0.15:  sm.l.diff_y= 0.15
    #sm.l.x_init[0]= sm.l.x_init3[0]+diff[0]
    sm.l.x_init[1]= sm.l.x_init3[1]+sm.l.diff_y
    CPrint(1,'modify..',sm.l.diff_y, vel_y)
    time.sleep(sm.l.flow_control_time_step)

  sm.NewState('init')
  sm['init'].EntryAction= Init
  sm['init'].ElseAction.Condition= lambda: True
  sm['init'].ElseAction.NextState= 'spread'

  sm.NewState('spread')
  sm['spread'].NewAction()
  sm['spread'].Actions[-1].Condition= lambda: not sm.ThreadInfo.IsRunning()
  sm['spread'].Actions[-1].NextState= EXIT_STATE
  sm['spread'].NewAction()
  sm['spread'].Actions[-1]= poured_action
  sm['spread'].NewAction()
  sm['spread'].Actions[-1]= timeout_action
  sm['spread'].ElseAction.Condition= lambda: True
  sm['spread'].ElseAction.Action= SpreadCtrlStep
  sm['spread'].ElseAction.NextState= ORIGIN_STATE

  return sm


#Control left arm for spreading (follow a trajectory)
def GenSMSpreadCtrl2(t,local_obj):
  sm= TStateMachine(debug=False, local_obj=local_obj)
  #sm.EventCallback= t.SMCallback  #WARNING: No callback to avoide confuse the context

  poured_action= TFSMConditionedAction()
  poured_action.Condition= sm.l.IsPoured
  poured_action.NextState= 'stop'

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= 'stop'

  sm.StartState= 'init'

  def Init():
    sm.l.x_init3= copy.deepcopy(sm.l.x_init)
    sm.l.diff_y= 0.0
    data= [ [0.0,0.0,0.0,0.0],
            [0.5, 0.05,0.02,0.0],
            [1.5,-0.05,0.07,0.0],
            [2.5, 0.05,0.12,0.0],
            [3.5,-0.05,0.17,0.0],
            [4.0, 0.0,0.19,0.0] ]
    sm.l.splines= [TCubicHermiteSpline() for d in range(len(data[0])-1)]
    for d in range(len(sm.l.splines)):
      data_d= [[x[0],x[d+1]] for x in data]
      sm.l.splines[d].Initialize(data_d, tan_method=sm.l.splines[d].CARDINAL, c=0.0, m=0.0)
    sm.l.spread_trjpoints= data
    sm.l.spread_time= sm.l.spread_trjpoints[0][0]  #Actual time
    sm.l.spread_itime= sm.l.spread_trjpoints[0][0]  #Internal time
    sm.l.spread_T= sm.l.spread_trjpoints[-1][0]  #Terminal time
    sm.l.spread_trj= lambda s: [sm.l.splines[d].Evaluate(s) for d in range(len(sm.l.splines))]
    sm.l.viz= TSimpleVisualizer(rospy.Duration(80.0), name_space='visualizer_flowc')
    sm.l.spread_step= 0

  def SpreadCtrlStep():
    dt= sm.l.flow_control_time_step
    itime= sm.l.spread_itime
    #Compute tracking velocity from flow
    vel= 0.5*sm.l.flow_sensor.FilteredFlowA(filter_size=5)
    if vel>0.03:  vel= 0.03
    if vel<0.0:   vel= 0.0
    #Compute target point (diff), get a new internal time (itime)
    itime,diff,v= ModifyTrajVelocityV(itime, vel, sm.l.spread_trj, dt, sm.l.spread_T)
    sm.l.x_init[:3]= [sm.l.x_init3[d]+diff[d] for d in range(3)]
    CPrint(1,'modify..', diff, vel)
    if sm.l.spread_step%10==0:
      if vel>0.0:
        sm.l.viz.AddSphere(sm.l.x_init, scale=[0.1*vel]*3, rgb=sm.l.viz.ICol(3), alpha=0.2)
    sm.l.spread_step+= 1
    time.sleep(dt)
    sm.l.spread_time+= dt
    sm.l.spread_itime= itime

  def Stop():
    #sm.l.viz.DeleteAllMarkers()
    #sm.l.viz= None
    pass

  sm.NewState('init')
  sm['init'].EntryAction= Init
  sm['init'].ElseAction.Condition= lambda: True
  sm['init'].ElseAction.NextState= 'spread'

  sm.NewState('spread')
  sm['spread'].NewAction()
  sm['spread'].Actions[-1].Condition= lambda: not sm.ThreadInfo.IsRunning()
  sm['spread'].Actions[-1].NextState= 'stop'
  sm['spread'].NewAction()
  sm['spread'].Actions[-1]= poured_action
  sm['spread'].NewAction()
  sm['spread'].Actions[-1]= timeout_action
  sm['spread'].ElseAction.Condition= lambda: True
  sm['spread'].ElseAction.Action= SpreadCtrlStep
  sm['spread'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('stop')
  sm['stop'].EntryAction= Stop
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.NextState= EXIT_STATE

  return sm


#Control left arm for spreading (follow a planned trajectory)
def GenSMSpreadCtrl3(t,local_obj):
  sm= TStateMachine(debug=False, local_obj=local_obj)
  #sm.EventCallback= t.SMCallback  #WARNING: No callback to avoide confuse the context

  poured_action= TFSMConditionedAction()
  poured_action.Condition= sm.l.IsPoured
  poured_action.NextState= 'stop'

  spreaded_action= TFSMConditionedAction()
  spreaded_action.Condition= lambda: sm.l.is_spreaded
  spreaded_action.NextState= 'stop'

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= sm.l.IsTimeout
  timeout_action.NextState= 'stop'

  sm.StartState= 'init'

  def Init():
    sm.l.x_init3= copy.deepcopy(sm.l.x_init)
    #start= t.GetAttr(CURR,'spread','xy_start')
    u_dir= t.GetAttr(CURR,'spread','xy_dir')
    rot= np.array([[u_dir[0],-u_dir[1]],[u_dir[1],u_dir[0]]])
    wave= t.GetAttr(CURR,'spread','wave')
    sm.l.spread_duration= t.GetAttr(CURR,'spread','duration')
    sm.l.spread_time= 0.0  #Actual time
    sm.l.spread_itime= 0.0  #Internal time
    sm.l.spread_trj= lambda s: np.dot(rot, wave.Evaluate(s))
    sm.l.viz= TSimpleVisualizer(rospy.Duration(80.0), name_space='visualizer_flowc')
    sm.l.spread_step= 0
    sm.l.log_file_spreadc= sm.l.get_local_tmp('flowcSP')
    sm.l.logfp_spreadc= file(sm.l.log_file_spreadc,'w')

  def SpreadCtrlStep():
    dt= sm.l.flow_control_time_step
    itime= sm.l.spread_itime
    #Compute tracking velocity from flow
    flow= sm.l.flow_sensor.FilteredFlowA(filter_size=5)
    vel= 1.0*flow
    if vel>0.05:  vel= 0.05
    if vel<0.0:   vel= 0.0
    #Compute target point (diff), get a new internal time (itime)
    itime,diff,v= ModifyTrajVelocityV(itime, vel, sm.l.spread_trj, dt, sm.l.spread_duration)
    sm.l.x_init[:2]= [sm.l.x_init3[d]+diff[d] for d in range(2)]
    CPrint(1,'Move xy..', diff, vel)
    if sm.l.spread_step%10==0:
      if vel>0.0:
        sm.l.viz.AddSphere(sm.l.x_init, scale=[0.1*vel]*3, rgb=sm.l.viz.ICol(3), alpha=0.2)
    sm.l.spread_step+= 1
    sm.l.logfp_spreadc.write('%f %f %f %f %f %f %f\n'%(rospy.Time.now().to_nsec(),
                                                    sm.l.spread_time,
                                                    sm.l.spread_itime,
                                                    flow,
                                                    vel,
                                                    diff[0], diff[1]))

    time.sleep(dt)
    sm.l.spread_time+= dt
    sm.l.spread_itime= itime
    if sm.l.spread_itime >= sm.l.spread_duration:
      sm.l.is_spreaded= True

  def Stop():
    #sm.l.viz.DeleteAllMarkers()
    #sm.l.viz= None
    sm.l.logfp_spreadc.close()
    CPrint(3,'Wrote spreading data to %s'%sm.l.log_file_spreadc)

  sm.NewState('init')
  sm['init'].EntryAction= Init
  sm['init'].ElseAction.Condition= lambda: True
  sm['init'].ElseAction.NextState= 'spread'

  sm.NewState('spread')
  sm['spread'].NewAction()
  sm['spread'].Actions[-1].Condition= lambda: not sm.ThreadInfo.IsRunning()
  sm['spread'].Actions[-1].NextState= 'stop'
  sm['spread'].NewAction()
  sm['spread'].Actions[-1]= poured_action
  sm['spread'].NewAction()
  sm['spread'].Actions[-1]= spreaded_action
  sm['spread'].NewAction()
  sm['spread'].Actions[-1]= timeout_action
  sm['spread'].ElseAction.Condition= lambda: True
  sm['spread'].ElseAction.Action= SpreadCtrlStep
  sm['spread'].ElseAction.NextState= ORIGIN_STATE

  sm.NewState('stop')
  sm['stop'].EntryAction= Stop
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.NextState= EXIT_STATE

  return sm
