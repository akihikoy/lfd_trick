#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move back from an object after grabbing.
  Usage: postgrab OBJ_ID
    OBJ_ID: identifier of object. e.g. 'b1'
  '''
def Run(t,*args):
  obj= args[0] if len(args)>0 else t.GetAttr(CURR,'source')

  if t.HasAttr(obj,'grabbed'):
    print 'Error: object is still grabbed: ',obj
    return FAILURE_PRECOND

  m_ctrl= t.LoadMotion('ctrl')

  l= TContainer(debug=True)
  l.obj= obj  #TODO: this should be stored
  l.handid= t.GetAttr(CURR,'init_handid')
  l.lw_xe= t.GetAttr('wrist_'+LRToStrs(l.handid),'lx')  #TODO: this should be stored
  l.conservative= t.GetAttrOr(False, CURR,'conservative')

  #def GetXGrab0():
    #t.ExecuteMotion('infer', l.obj,'x')
    #x_o= t.GetAttr(l.obj,'x')
    #lo_x_grab0= t.GetAttr(l.obj,'l_x_grab0')
    #x_grab0= Transform(x_o,lo_x_grab0)
    #return x_grab0

  #l.shift_to_grab0= m_ctrl.TApproachToX()
  #l.shift_to_grab0.core_tool= t
  #l.shift_to_grab0.get_x_trg= GetXGrab0
  #l.shift_to_grab0.l_x_ext= l.lw_xe
  #l.shift_to_grab0.arm= l.handid  #TODO: this should be stored

  def WithdrawX(x_curr):
    p,R= XToPosRot(x_curr)
    g_width= t.GetAttr(l.obj,'g_width')
    p= p - 2.0*g_width*R[:,0]  #Withdraw to ex direction
    p[2]+= 0.04  #4cm above; hack to avoid collision of arm with table
    return PosRotToX(p,R)

  l.shift_to_grab0= m_ctrl.TMoveDiffX()
  l.shift_to_grab0.core_tool= t
  l.shift_to_grab0.change_x= WithdrawX
  l.shift_to_grab0.l_x_ext= l.lw_xe
  l.shift_to_grab0.arm= l.handid
  #Move until collision is free or reaching WithdrawX
  l.shift_to_grab0.init_callback= lambda:t.ExecuteMotion('scene', 'make',[],1.3)  #1.3 is bigger margin than that is used in infer_traj (called from move_to_x)
  l.shift_to_grab0.additional_check= lambda:not (t.ExecuteMotion('scene','isvalidq',l.handid))[0]
  l.shift_to_grab0.exit_callback= lambda:t.ExecuteMotion('scene', 'clear')

  def MoveToInit():
    x_init= t.GetAttr(CURR,'init_x')
    l.exec_status= t.ExecuteMotion('move_to_x', x_init, 3.0, l.lw_xe, l.handid, {}, l.conservative)


  sm= TStateMachine()
  sm.EventCallback= t.SMCallback
  sm.Debug= True

  sm.StartState= 'start'
  sm.NewState('start')
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'shift_to_grab0'

  sm.NewState('shift_to_grab0')
  sm['shift_to_grab0'].EntryAction= l.shift_to_grab0.Init
  sm['shift_to_grab0'].NewAction()
  sm['shift_to_grab0'].Actions[-1].Condition= l.shift_to_grab0.Check
  sm['shift_to_grab0'].Actions[-1].Action= l.shift_to_grab0.Step
  sm['shift_to_grab0'].Actions[-1].NextState= ORIGIN_STATE
  sm['shift_to_grab0'].ElseAction.Condition= lambda: True
  sm['shift_to_grab0'].ElseAction.Action= l.shift_to_grab0.Exit
  sm['shift_to_grab0'].ElseAction.NextState= 'move_to_init'

  sm.NewState('move_to_init')
  sm['move_to_init'].EntryAction= MoveToInit
  sm['move_to_init'].NewAction()
  sm['move_to_init'].Actions[-1].Condition= lambda:IsSuccess(l.exec_status)
  sm['move_to_init'].Actions[-1].NextState= EXIT_STATE
  sm['move_to_init'].ElseAction.Condition= lambda:True
  sm['move_to_init'].ElseAction.Action= sm.SetFailure
  sm['move_to_init'].ElseAction.NextState= EXIT_STATE


  t.RunSM(sm,'postgrab')
  l= None


  return sm.ExitStatus
