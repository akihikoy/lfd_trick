#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move to a Cartesian pose with trajectory planning.
  Usage: move_to_x X_TRG, DURATION, X_EXT, HAND_ID [, SITUATION [, CONSERVATIVE]]
    X_TRG: target pose
    DURATION: duration of motion
    X_EXT: task space in wrist frame
    HAND_ID: hand id (defined in base_const.py)
    SITUATION: extra parameters for inter2::traj (default: {})
    CONSERVATIVE: robot behaves conservatively (default: False)
  '''
def Run(t,*args):
  x_trg= args[0]
  duration= args[1]
  x_ext= args[2]
  hand_id= args[3]
  situation= args[4] if len(args)>4 else {}
  conservative= args[5] if len(args)>5 else False

  #situation['objs']= [bottle,cup]
  situation['handid']= hand_id
  situation['l_x_ext']= x_ext
  situation['x_target']= x_trg
  situation['dt']= duration
  #situation['N']= 40
  #situation['bb_margin']= 1.0

  t.DelAttr(CURR,'q_traj')
  t.DelAttr(CURR,'t_traj')
  t.DelAttr(CURR,'x_traj')
  t.ExecuteMotion('infer2', situation,[],'traj')

  if not t.HasAttr(CURR,'q_traj') or not t.HasAttr(CURR,'t_traj'):
    print 'Failed to infer q_traj/t_traj'
    return FailureCode('infer_q_traj')
  if conservative:
    print 'Follow the trajectory?'
    if not t.AskYesNo():  return FailureCode('canceled')


  def AssessTraj(a, event_type, context, sm):
    MemorizeAsBad= lambda assess_key, assess_info, ask=True: \
        t.MemorizeAssessment(a, mem_key=['traj','bad'], assess_key=assess_key, assess_info=assess_info, context=copy.deepcopy(context), ask=ask)

    exception_req= False
    if t.GetAttrOr(False, CURR,'human_assessed_bad') and not a.MarkedBad:
      CPrint(3, 'Was the inference "traj" wrong?')
      if t.AskYesNo():
        if MemorizeAsBad(assess_key='human_assess', assess_info={'message':'bad'}, ask=False):
          exception_req= True

    if event_type==a.EVENT_EXIT and context==a.RegisteredContext:
      a.Conditions= set()

    if t.GetAttrOr(False, CURR,'human_assessed_bad') and not exception_req:
      CPrint(3, 'Stop the motion now? (EXCEPTION)')
      if t.AskYesNo():  exception_req= True

    if exception_req:  return a.EXCEPTION
    return a.COMPLETED if len(a.Conditions)==0 else a.CONTINUE

  assess_traj= TAssessment(TAssessment.STATE_CALLBACK, name='assess_traj')
  assess_traj.Assess= AssessTraj
  assess_traj.Conditions= set(['traj'])
  assess_traj.DBIndex= t.GetAttr(CURR,'new_in_database')[-1]
  assess_traj.MarkedBad= False
  t.RegisterAssessment(assess_traj)

  q_traj= t.GetAttr(CURR,'q_traj')
  t_traj= t.GetAttr(CURR,'t_traj')
  print 'q_traj',q_traj
  print 't_traj',t_traj
  LimitQTrajVel(q_start=t.robot.Q(hand_id), q_traj=q_traj, t_traj=t_traj, qvel_limits=t.robot.JointVelLimits(hand_id))
  print 'Modified q_traj',q_traj
  print 'Modified t_traj',t_traj
  t.robot.FollowQTraj(q_traj, t_traj, arm=hand_id, blocking=True)

  return SUCCESS_CODE
