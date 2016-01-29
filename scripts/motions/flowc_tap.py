#!/usr/bin/python
from core_tool import *
def Help():
  return '''Flow amount controller with tapping.  WARNING: NOT MAINTAINED.
  Assumptions:
    Left gripper holds a bottle
    Bottle is close to the cup
    Right gripper is a ready pose for tapping
  Usage: flowc_tap BOTTLE_ID [, AMOUNT_TRG [, MAX_DURATION]]
    BOTTLE_ID: identifier of bottle. e.g. 'b1'
    AMOUNT_TRG: Target amount (default=0.03)
    MAX_DURATION: Maximum duration (default=25.0)'''
def Run(t,*args):
  bottle= args[0]
  amount_trg= args[1] if len(args)>1 else 0.03
  max_duration= args[2] if len(args)>2 else 25.0

  raise Exception('flowc_tap:FIXME: replace flowc_cmn by flowc_cmn3')
  m_flowc_cmn= t.LoadMotion('flowc_cmn')
  l= m_flowc_cmn.TLocal()
  if not l.Setup(t, bottle,amount_trg,max_duration):
    return FAILURE_PRECOND

  l.m_infer= t.LoadMotion('infer')
  #Tapping pose in bottle frame
  l.lb_x_tap= t.GetAttr(l.bottle,'l_x_tap')

  l.x_r_ext= t.GetAttr('wrist_r','lx')
  l.x_r_init= np.array(t.robot.FK(x_ext=l.x_r_ext, arm=RIGHT))

  l.flow_obs_sensitivity= 0.003 #FIXME

  l.behavior_type= 'tap'

  def MoveRGripperToTap():
    l.ctrl_type= 'mrt'
    print 'Moving R-gripper to the tapping pose...'
    l.tmpfp.write('%f %f %f %f %f %f mrt\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))

    l.m_infer.Run(t, l.bottle,'x')
    x_b= t.GetAttr(l.bottle,'x')  #Bottle pose in robot frame
    l.x_tap= Transform(x_b,l.lb_x_tap)  #Tapping pose in robot frame

    t.robot.MoveToXI(l.x_tap,3.0,l.x_r_ext,inum=30, arm=RIGHT,blocking='time')
    l.elapsed_time+= 3.0

    l.tmpfp.write('%f %f %f %f %f %f mrt\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))

  def MoveRGripperToInit():
    l.ctrl_type= 'mri'
    print 'Moving R-gripper to the initial pose...'
    l.tmpfp.write('%f %f %f %f %f %f mri\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))
    t.robot.MoveToXI(l.x_r_init,3.0,l.x_r_ext,inum=30, arm=RIGHT,blocking='time')
    l.elapsed_time+= 3.0
    l.tmpfp.write('%f %f %f %f %f %f mri\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))

  def Vibrate(count,dt=0.4):
    l.ctrl_type= 'tp'
    l.amount_prev= l.amount
    l.amount= t.material_amount

    print 'vibrate:',l.elapsed_time,': ',l.amount,' / ',l.amount_trg #,' : ',dt
    #x_r= np.array(t.robot.FK(x_ext=l.x_r_ext, arm=RIGHT))
    x_r= l.x_tap
    x_r_trg= copy.deepcopy(x_r)
    x_r_trg[2]-= 0.018  #FIXME: magic number!!!
    for i in range(count):
      t.robot.MoveToXI(x_r_trg,dt/2.0,l.x_r_ext,inum=5, arm=RIGHT,blocking='time')
      l.tmpfp.write('%f %f %f %f %f %f tp\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))
      t.robot.MoveToXI(x_r,dt/2.0,l.x_r_ext,inum=5, arm=RIGHT,blocking='time')
      l.tmpfp.write('%f %f %f %f %f %f tp\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))
      l.elapsed_time+= dt

  def Repeat(duration, action):
    l.ChargeTimer(duration)
    while not l.IsTimerTimeout():
      action()

  def LVibrate(count,dt=0.01):
    l.ctrl_type= 'vc'
    l.amount_prev= l.amount
    l.amount= t.material_amount

    print 'l-vibrate:',l.elapsed_time,': ',l.amount,' / ',l.amount_trg #,' : ',dt
    x_trg1= copy.deepcopy(l.x_init2)
    x_trg2= copy.deepcopy(l.x_init2)
    x_trg2[0]+= 0.005  #FIXME: magic number!!!
    for i in range(count):
      t.robot.MoveToXI(x_trg1,dt/2.0,l.x_ext,inum=5, arm=LEFT,blocking='time')
      l.tmpfp.write('%f %f %f %f %f %f vc\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))
      t.robot.MoveToXI(x_trg2,dt/2.0,l.x_ext,inum=5, arm=LEFT,blocking='time')
      l.tmpfp.write('%f %f %f %f %f %f vc\n' % (rospy.Time.now().to_nsec(),t.material_amount,l.amount_trg,l.amount_trg,-999,-999))
      l.elapsed_time+= dt

  sm= TStateMachine()
  sm.EventCallback= t.SMCallback
  sm.Debug= True

  timeout_action= TFSMConditionedAction()
  timeout_action.Condition= l.IsTimeout
  timeout_action.NextState= 'stop'

  poured_action= TFSMConditionedAction()
  poured_action.Condition= l.IsPoured
  poured_action.NextState= 'stop'

  move_back_action= lambda: Repeat(0.3,lambda: l.ControlStep(0.5 * l.flow_control_dtheta_min))

  sm.StartState= 'start'
  sm['start']= TFSMState()
  sm['start'].NewAction()
  sm['start'].Actions[-1]= poured_action
  sm['start'].NewAction()
  sm['start'].Actions[-1]= timeout_action
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: l.IsFlowObserved(l.flow_obs_sensitivity)
  #sm['start'].Actions[-1].Action= move_back_action
  sm['start'].Actions[-1].NextState= 'move_r_tap'
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: l.theta>(l.max_theta-0.5*math.pi)
  sm['start'].Actions[-1].NextState= 'move_r_tap'
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.Action= lambda: l.ControlStep(0.2 * l.flow_control_dtheta_max)
  sm['start'].ElseAction.NextState= ORIGIN_STATE

  sm['move_r_tap']= TFSMState()
  sm['move_r_tap'].EntryAction= MoveRGripperToTap
  sm['move_r_tap'].NewAction()
  sm['move_r_tap'].Actions[-1]= poured_action
  sm['move_r_tap'].NewAction()
  sm['move_r_tap'].Actions[-1]= timeout_action
  sm['move_r_tap'].ElseAction.Condition= lambda: True
  sm['move_r_tap'].ElseAction.NextState= 'tap'

  sm['tap']= TFSMState()
  sm['tap'].EntryAction= lambda: l.ChargeTimer(4.0)
  sm['tap'].NewAction()
  sm['tap'].Actions[-1]= poured_action
  sm['tap'].NewAction()
  sm['tap'].Actions[-1]= timeout_action
  sm['tap'].NewAction()
  sm['tap'].Actions[-1].Condition= lambda: l.IsFlowObserved(l.flow_obs_sensitivity)
  sm['tap'].Actions[-1].Action= lambda: (l.ChargeTimer(4.0), Vibrate(3))
  sm['tap'].Actions[-1].NextState= ORIGIN_STATE
  sm['tap'].NewAction()
  sm['tap'].Actions[-1].Condition= lambda: not l.IsTimerTimeout()
  sm['tap'].Actions[-1].Action= lambda: Vibrate(3)
  sm['tap'].Actions[-1].NextState= ORIGIN_STATE
  sm['tap'].ElseAction.Condition= lambda: True
  sm['tap'].ElseAction.NextState= 'move_r_init'

  sm['move_r_init']= TFSMState()
  sm['move_r_init'].EntryAction= MoveRGripperToInit
  sm['move_r_init'].NewAction()
  sm['move_r_init'].Actions[-1]= poured_action
  sm['move_r_init'].NewAction()
  sm['move_r_init'].Actions[-1]= timeout_action
  sm['move_r_init'].ElseAction.Condition= lambda: True
  sm['move_r_init'].ElseAction.NextState= 'pour'

  sm['pour']= TFSMState()
  sm['pour'].EntryAction= lambda: l.ChargeTimer(4.0)
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= poured_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1]= timeout_action
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= lambda: l.IsFlowObserved(l.flow_obs_sensitivity)
  #sm['pour'].Actions[-1].Action= move_back_action
  sm['pour'].Actions[-1].NextState= 'move_r_tap'
  sm['pour'].NewAction()
  sm['pour'].Actions[-1].Condition= lambda: l.IsTimerTimeout()
  sm['pour'].Actions[-1].NextState= 'move_r_tap'
  sm['pour'].ElseAction.Condition= lambda: True
  sm['pour'].ElseAction.Action= lambda: l.ControlStep(0.05 * l.flow_control_dtheta_max)  #FIXME: magic number
  sm['pour'].ElseAction.NextState= ORIGIN_STATE

  sm['stop']= TFSMState()
  #sm['stop'].EntryAction= lambda: (MoveRGripperToInit(), l.MoveBackToInit())
  #sm['stop'].ElseAction.Condition= lambda: True
  #sm['stop'].ElseAction.Action= lambda: Print('End of pouring')
  #sm['stop'].ElseAction.NextState= EXIT_STATE
  sm['stop'].EntryAction= lambda: MoveRGripperToInit()
  sm['stop'].NewAction()
  sm['stop'].Actions[-1].Condition= lambda: l.IsThetaEqTo(0.0)
  sm['stop'].Actions[-1].Action= lambda: Print('End of pouring')
  sm['stop'].Actions[-1].NextState= EXIT_STATE
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.Action= lambda: l.ControlStep(l.flow_control_dtheta_min)
  sm['stop'].ElseAction.NextState= ORIGIN_STATE

  sm.Run()

  l.Close()
  l= None

  return sm.ExitStatus
