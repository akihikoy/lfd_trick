#!/usr/bin/python
from core_tool import *
def Help():
  return '''Handle the virtual scene for planning.
  Usage:
    scene
    scene 'clear'
      Clear the virtual scene to default
    scene 'make' [, IGN_OBJS [, BB_MARGIN [, SITUATION]]]
      Construct the virtual scene with putting objects described in attribute [CURR]['scene']
      IGN_OBJS: A list of objects to be ignored (default: [])
      BB_MARGIN: Margin ratio of a bounding box (default: 1.1)
      SITUATION: Situation dict to return the object information (default: N/A)
    scene 'isvalidq', ARM [, IGN_OBJS [, Q [, VISUALIZE]]]
      Check the validity of joint angles
      ARM: Hand ID
      IGN_OBJS: A list of objects to be ignored (default: [])
      Q: Joint angles (default: t.robot.Q(ARM))
      VISUALIZE: Visualize or not (default: False)
      Return: validity, result of scene checker
        Note: IGN_OBJS will be included in the result of scene checker
    scene 'isvalidx', ARM, X [, IGN_OBJS [, Q_INIT [, VISUALIZE]]]
      Check the validity of Cartesian pose in the wrist frame
      ARM: Hand ID
      X: Cartesian pose
      IGN_OBJS: A list of objects to be ignored (default: [])
      Q_INIT: Initial joint angles to solve for X (default: t.robot.Q(ARM))
      VISUALIZE: Visualize or not (default: False)
      Return: validity, result of IK checker, result of scene checker
        Note: IGN_OBJS will be included in the result of scene checker
  '''
def Run(t,*args):
  res= True
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='clear':
    CPrint(3, 'Clearing scene')
    with t.robot.sensor_locker:
      t.state_validity_checker.RemoveFromScene()
    if 'scene' in t.viz:
      del t.viz['scene']
  elif command=='make':
    ign_objs= set(args[0]) if len(args)>0 else set()
    bb_margin= args[1] if len(args)>1 else 1.1
    situation= args[2] if len(args)>2 else {}

    #Load scene:
    objs= set(t.GetAttrOr([], CURR,'scene'))
    cmn= objs&ign_objs  #Common in objs and ign_objs
    objs-= cmn
    ign_objs-= cmn

    CPrint(3, 'Making scene:',objs,'ignoring:',ign_objs,'margin:',bb_margin)

    t.viz.scene= TSimpleVisualizer(name_space='visualizer_scene')
    t.viz.scene.viz_frame= t.robot.BaseFrame

    t.state_validity_checker.InitToMakeScene()

    with t.robot.sensor_locker:
      x_w_lr= [t.robot.FK(arm=RIGHT), t.robot.FK(arm=LEFT)]
      #Add physical collision models
      if t.robot.Is('Baxter'):
        for wobj in ('wrist_r','wrist_l','wl_m100'):
          bound_box= t.GetAttr(wobj, 'bound_box')
          arm= StrToLR(t.GetAttr(wobj, 'kinematics_chain','parent_arm'))
          lw_xe= t.GetAttr(wobj,'lx')
          xe= Vec(Transform(x_w_lr[arm],lw_xe))
          lx_bb= bound_box['center']
          dim_bb= Vec(bound_box['dim'])*bb_margin
          x_bb= Transform(xe, lx_bb)
          lwx_bb= TransformLeftInv(x_w_lr[arm], x_bb)
          t.state_validity_checker.AddBoxToRobotHand(lwx_bb,dim_bb,arm=arm,name=wobj)
          t.viz.scene.AddCube(x_bb, dim_bb, rgb=t.viz.scene.ICol(1), alpha=0.3)
          CPrint(3, '  attaching %r to %s-hand'%(wobj,LRToStr(arm)))
      #Add virtual collision models
      for obj in objs:
        t.ExecuteMotion('infer2', {},[obj],'x')
        if not t.HasAttr(obj,'x'):
          CPrint(4,'Error: cannot add %r to scene' % obj)
          res= False
          continue
        x_o= t.GetAttr(obj,'x')
        t.ExecuteMotion('infer2', {},[obj],'bound_box')
        if not t.HasAttr(obj, 'bound_box'):
          CPrint(4,'Error: cannot get the bound box of %r' % obj)
          res= False
          continue
        bound_box= t.GetAttr(obj, 'bound_box')
        lo_o_center= bound_box['center']
        o_dim= Vec(bound_box['dim'])*bb_margin
        o_center= Transform(x_o, lo_o_center)
        if t.HasAttr(obj,'grabbed'):
          grab_arm= t.GetAttr(obj,'grabbed','grabber_handid')
          lw_o_center= TransformLeftInv(x_w_lr[grab_arm],o_center)
          t.state_validity_checker.AddBoxToRobotHand(lw_o_center,o_dim,arm=grab_arm,name=obj)
          InsertDict(situation, {'objs_attr':{obj:{'x':x_o,'grabbed':{'grabber_handid':grab_arm}}}})
          CPrint(3, '  attaching %r to %s-hand'%(obj,LRToStr(grab_arm)))
        else:
          t.state_validity_checker.AddBoxToScene(o_center,o_dim,name=obj)
          InsertDict(situation, {'objs_attr':{obj:{'x':x_o}}})
          CPrint(3, '  adding %r to scene'%(obj))
        for obj2 in ign_objs:
          CPrint(3, '  ignoring collision between %r and %r'%(obj,obj2))
          t.state_validity_checker.IgnoreCollision(obj,obj2)
        if t.robot.Is('PR2'):
          #Ignore collision of obj with 'collision_map'
          CPrint(3, '  ignoring collision between %r and %r'%(obj,'collision_map'))
          t.state_validity_checker.IgnoreCollision(obj,'collision_map')
        t.viz.scene.AddCube(o_center, o_dim, rgb=t.viz.scene.ICol(1), alpha=0.3)
      for obj2 in ign_objs:
        CPrint(3, '  ignoring collision between %r and %r'%('all',obj2))
        t.state_validity_checker.IgnoreCollision('all',obj2)
      #Ignoring list
      if t.robot.Is('PR2'):
        ign_combinations= (('all','collision_map'),
                ('.robot','collision_map'),
                ('.l_gripper','l_gripper_sensor_mount_link'))
      elif t.robot.Is('Baxter'):
        ign_combinations= (('display','.head'),
                ('right_gripper_base','.r_gripper'),
                ('right_upper_elbow_visual','torso'),
                ('left_upper_elbow_visual','torso'))
      for i1,i2 in ign_combinations:
        CPrint(3, '  ignoring collision between %r and %r'%(i1,i2))
        t.state_validity_checker.IgnoreCollision(i1,i2)

      t.state_validity_checker.SendToServer()
      #print xxx.planning_scene.link_padding

  elif command=='isvalidq':
    arm= args[0]
    ign_objs= set(args[1]) if len(args)>1 else set()
    q= args[2] if len(args)>2 else t.robot.Q(arm)
    visualization= args[3] if len(args)>3 else False

    with t.robot.sensor_locker:
      res= t.state_validity_checker.IsValidState(q, arm=arm)
    if visualization:  VisualizeContacts(res.contacts)
    if isinstance(t.state_validity_checker, TStateValidityChecker):
      if res.error_code.val==res.error_code.SUCCESS:
        return True, res
      elif res.error_code.val==res.error_code.COLLISION_CONSTRAINTS_VIOLATED:
        col_objs= GetCollidingObjects(res.contacts, ign_objs)
        return len(col_objs)==0, res
      else:
        return False, res
    elif isinstance(t.state_validity_checker, TStateValidityCheckerMI):
      if res.valid:
        return True, res
      else:
        col_objs= GetCollidingObjects(res.contacts, ign_objs)
        return len(col_objs)==0, res
    else:
      raise Exception('Unknown t.state_validity_checker type:',type(t.state_validity_checker))

  elif command=='isvalidx':
    arm= args[0]
    x= args[1]
    ign_objs= set(args[2]) if len(args)>2 else set()
    q_init= args[3] if len(args)>3 else t.robot.Q(arm)
    visualization= args[4] if len(args)>4 else False
    #print '###DEBUG### \'isvalidx\'',','.join(map(str,args))

    q1,res1= t.robot.IK(x, start_angles=q_init, arm=arm, with_st=True)
    if q1==None:
      #print '###DEBUG### resD:',False, res1, None
      return False, res1, None
    with t.robot.sensor_locker:
      res2= t.state_validity_checker.IsValidState(q1, arm=arm)
    if visualization:  VisualizeContacts(res2.contacts)
    if isinstance(t.state_validity_checker, TStateValidityChecker):
      if res2.error_code.val==res2.error_code.SUCCESS:
        #print '###DEBUG### resA:',True, res1, res2
        return True, res1, res2
      elif res2.error_code.val==res2.error_code.COLLISION_CONSTRAINTS_VIOLATED:
        col_objs= GetCollidingObjects(res2.contacts, ign_objs)
        #print '###DEBUG### resB:',len(col_objs)==0, res1, res2
        return len(col_objs)==0, res1, res2
      else:
        #print '###DEBUG### resC:',False, res1, res2
        return False, res1, res2
    elif isinstance(t.state_validity_checker, TStateValidityCheckerMI):
      if res2.valid:
        return True, res1, res2
      else:
        col_objs= GetCollidingObjects(res2.contacts, ign_objs)
        return len(col_objs)==0, res1, res2

  else:
    raise Exception('Invalid command: %r'%command)

  return res
