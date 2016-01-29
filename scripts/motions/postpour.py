#!/usr/bin/python
from core_tool import *
def Help():
  return '''Post-pouring motion where the bottle should be grabbed.
  Usage: postpour BOTTLE_ID, CUP_ID
    BOTTLE_ID: identifier of bottle. e.g. 'b1'
    CUP_ID: identifier of cup. e.g. 'c1' '''
def Run(t,*args):
  bottle= args[0] if len(args)>0 else t.GetAttr(CURR,'source')
  cup= args[1] if len(args)>1 else t.GetAttr(CURR,'receiver')

  if not t.HasAttr(bottle,'grabbed'):
    print 'Error: not grabbed: ',bottle
    return FAILURE_PRECOND

  m_ctrl= t.LoadMotion('ctrl')

  l= TContainer(debug=True)
  l.bottle= bottle
  l.cup= cup
  l.handid= t.GetAttr(l.bottle,'grabbed','grabber_handid')
  l.lw_xe= t.GetAttr('wrist_'+LRToStrs(l.handid),'lx')
  l.conservative= t.GetAttrOr(False, CURR,'conservative')


  l.shift_to_pour_l0= m_ctrl.TApproachToX()
  l.shift_to_pour_l0.core_tool= t
  l.shift_to_pour_l0.get_x_trg= lambda: t.GetAttr(CURR,'x_pour_l0')
  l.shift_to_pour_l0.l_x_ext= t.GetAttr(CURR,'lw_x_pour_e')
  l.shift_to_pour_l0.arm= l.handid
  #Finish moving if the state is collision free
  l.shift_to_pour_l0.init_callback= lambda:t.ExecuteMotion('scene', 'make',[],1.2)  #1.2 is bigger margin than that is used in infer_traj (called from move_to_x)
  l.shift_to_pour_l0.additional_check= lambda:not (t.ExecuteMotion('scene','isvalidq',l.handid))[0]
  l.shift_to_pour_l0.exit_callback= lambda:t.ExecuteMotion('scene', 'clear')

  def MoveToGrabbed0():
    x_b_trg= copy.deepcopy(t.GetAttr(l.bottle,'grabbed','grabbed_x'))

    #Control point of x_b
    t.ExecuteMotion('infer', l.bottle,'x')
    x_b= t.GetAttr(l.bottle,'x')
    x_w= t.robot.FK(arm=l.handid)
    lw_x_b= TransformLeftInv(x_w, x_b)

    #Plan a collision free pose above the grabbed_x
    x_b_trg2= copy.deepcopy(x_b_trg)
    t.ExecuteMotion('scene', 'make')
    for dz in FRange1(0.0, 0.10, 40):  #FIXME: magic parameter
      x_b_trg2[2]= x_b_trg[2]+dz
      vx,res1,res2= t.ExecuteMotion('scene','isvalidx',l.handid,TransformRightInv(x_b_trg2,lw_x_b))
      if vx:
        l.grabbed_0_dz= dz
        break
      #if res1!=None: print 'IK error_code:',res1.error_code.val,
      #if res2!=None: print 'Collision:',GetCollidingObjects(res2.contacts),
      #print ''
    t.ExecuteMotion('scene', 'clear')
    if 'grabbed_0_dz' not in l:
      CPrint(4,'Failed to infer grabbed_0_dz')
      l.exec_status= FailureCode('infer_grabbed_0_dz')
      return
    x_b_trg[2]+= l.grabbed_0_dz

    l.exec_status= t.ExecuteMotion('move_to_x', x_b_trg, 5.0, lw_x_b, l.handid, {}, l.conservative)


  def ChangeXDownward(x_curr):
    x_res= copy.deepcopy(x_curr)
    x_res[2]-= l.grabbed_0_dz
    return x_res

  l.move_downward= m_ctrl.TMoveDiffX()
  l.move_downward.core_tool= t
  l.move_downward.change_x= ChangeXDownward
  l.move_downward.l_x_ext= l.lw_xe
  l.move_downward.arm= l.handid
  l.move_downward.using_PI= False




  sm= TStateMachine()
  sm.EventCallback= t.SMCallback
  sm.Debug= True

  sm.StartState= 'start'
  sm.NewState('start')
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'shift_to_pour_l0'

  sm.NewState('shift_to_pour_l0')
  sm['shift_to_pour_l0'].EntryAction= l.shift_to_pour_l0.Init
  sm['shift_to_pour_l0'].NewAction()
  sm['shift_to_pour_l0'].Actions[-1].Condition= l.shift_to_pour_l0.Check
  sm['shift_to_pour_l0'].Actions[-1].Action= l.shift_to_pour_l0.Step
  sm['shift_to_pour_l0'].Actions[-1].NextState= ORIGIN_STATE
  sm['shift_to_pour_l0'].ElseAction.Condition= lambda: True
  sm['shift_to_pour_l0'].ElseAction.Action= l.shift_to_pour_l0.Exit
  sm['shift_to_pour_l0'].ElseAction.NextState= 'move_to_grabbed0'

  sm.NewState('move_to_grabbed0')
  sm['move_to_grabbed0'].EntryAction= MoveToGrabbed0
  sm['move_to_grabbed0'].NewAction()
  sm['move_to_grabbed0'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['move_to_grabbed0'].Actions[-1].NextState= 'move_downward'
  sm['move_to_grabbed0'].ElseAction.Condition= lambda: True
  sm['move_to_grabbed0'].ElseAction.Action= sm.SetFailure
  sm['move_to_grabbed0'].ElseAction.NextState= EXIT_STATE

  sm.NewState('move_downward')
  sm['move_downward'].EntryAction= l.move_downward.Init
  sm['move_downward'].NewAction()
  sm['move_downward'].Actions[-1].Condition= l.move_downward.Check
  sm['move_downward'].Actions[-1].Action= l.move_downward.Step
  sm['move_downward'].Actions[-1].NextState= ORIGIN_STATE
  sm['move_downward'].ElseAction.Condition= lambda: True
  sm['move_downward'].ElseAction.Action= l.move_downward.Exit
  sm['move_downward'].ElseAction.NextState= EXIT_STATE


  t.RunSM(sm,'postpour')
  l= None


  return sm.ExitStatus

