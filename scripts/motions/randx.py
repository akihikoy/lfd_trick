#!/usr/bin/python
from core_tool import *
def Help():
  return '''Randomly assign 'x' attribute.
  Usage: randx R_TYPE, OBJ_ID1 [, OBJ_ID2 ...]
    R_TYPE: type of 'x'. select from below
      'marker': AR-marker like value
      'markercf': AR-marker like value, collision free, IK reachable (with left hand)
      'aligned': put on a line at regular intervals
      'uni': uniform random: x,y,z is between [-1,-1,-1] and [1,1,1] and any q
    OBJ_ID*: identifier of object. e.g. 'b1'
  Usage: randx R_TYPE
    Return random x.
  '''
def GetX(t,r_type):
  if t.robot.Is('PR2'):  pc= [0.7, 0.0, -0.28]
  elif t.robot.Is('Baxter'):  pc= [0.7, 0.0, 0.0]
  rad= [0.15, 0.2, 0.0]
  rand1= lambda: 2.0*(random.random()-0.5)
  if r_type=='marker':
    x= [0.0]*7
    x[:3]= [pc[d]+rad[d]*rand1() for d in range(3)]
    x[3:]= QFromAxisAngle([0.0,0.0,1.0], math.pi*rand1())
    return x
  elif r_type=='uni':
    x= [0.0]*7
    x[:3]= [rand1() for d in range(3)]
    x[3:]= QFromAxisAngle([rand1() for d in range(3)], math.pi*rand1())
    return x
  raise
def Run(t,*args):
  r_type= args[0]
  if r_type not in ('marker','markercf','aligned','uni'):
    print 'Invalid type:',r_type
    print 'Help:',Help()
    return

  ik_arm= LEFT
  min_dist= 0.20
  objs= args[1:]
  if len(objs)==0:
    if r_type=='markercf':  return GetX(t,'marker')
    else:  return GetX(t,r_type)

  if r_type in ('marker','uni'):
    for obj in objs:
      x= GetX(t,r_type)
      t.SetAttr(obj,'x', x)
      print "['%s']['x']= %r" % (obj,x)
    return True
  elif r_type == 'markercf':
    success= False
    t.SetAttr(CURR,'scene', objs)
    for i in range(500):
      for obj in objs:
        x= GetX(t,'marker')
        t.SetAttr(obj,'x', x)

      #IK check
      lw_xe= t.GetAttr('wrist_'+LRToStrs(ik_arm),'lx')
      if t.robot.IK(x, x_ext=lw_xe, arm=ik_arm)==None:
        print i,'Not reachable (IK failure)'
        continue

      #Collision between objects
      obj_col= False
      for o1 in objs:
        x1= t.GetAttr(o1,'x')
        for o2 in objs:
          if o1==o2: continue
          x2= t.GetAttr(o2,'x')
          d= Norm(Vec(x2)[:3]-Vec(x1)[:3])
          print 'Dist',o1,o2,d
          if d<min_dist:
            obj_col= True
            break
        if obj_col:
          break
      if obj_col:
        print i,'Objects are colliding'
        continue

      #Collision with robots
      t.ExecuteMotion('scene', 'make', [], 2.0)
      v= t.ExecuteMotion('scene', 'isvalidq',0)
      if not v[0]:
        print i,'Collision detected:', GetCollidingObjects(v[1].contacts)
        continue

      success= True
      break

    t.ExecuteMotion('scene', 'clear')
    for obj in objs:
      print "['%s']['x']= %r" % (obj,t.GetAttr(obj,'x'))
    return success

  elif r_type=='aligned':
    pc= [1.0, 0.0, -0.3]
    q= [0.0,0.0,0.0,1.0]
    iv= 0.5
    pc[1]= -iv*float(len(objs)-1)/2.0
    for obj in objs:
      x= pc+q
      t.SetAttr(obj,'x', x)
      print "['%s']['x']= %r" % (obj,x)
      pc[1]+= iv
    return True

