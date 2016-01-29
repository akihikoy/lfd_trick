#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test the collision checker.
  Usage: test_col TEST_CASE
    TEST_CASE: "box" or "cyl"
  '''
def Run(t,*args):
  #test_case= 'box'
  #test_case= 'cyl'
  test_case= args[0] if len(args)>0 else 'box'

  l= TContainer(debug=True)
  l.ljscallback_init= True


  #Move the left arm to the base pose
  if t.robot.Is('PR2'):
    t.robot.MoveToQ([0.0]*7,dt=2.0, arm=RIGHT,blocking=False)
    t.robot.MoveToQ([0.0]*7,dt=2.0, arm=LEFT,blocking=True)
  elif t.robot.Is('Baxter'):
    t.robot.MoveToQ([math.pi/4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],dt=2.0, arm=RIGHT,blocking=False)
    t.robot.MoveToQ([-math.pi/4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],dt=2.0, arm=LEFT,blocking=True)


  t.viz.scene= TSimpleVisualizer(name_space='visualizer_scene')
  t.viz.scene.viz_frame= t.robot.BaseFrame

  def Close(t):
    #if 'l_ctrl_state' in t.sub:
      #t.sub.l_ctrl_state.unregister()
      #del t.sub.l_ctrl_state
    with t.robot.sensor_locker:
      t.state_validity_checker.RemoveFromScene()
    if 'scene' in t.viz:
      del t.viz['scene']

  t.state_validity_checker.InitToMakeScene()

  #Using Box:
  if test_case=='box':
    #Add an object to scene:
    l.o_dim1= [0.2, 0.2, 1.0]
    if t.robot.Is('PR2'):
      l.o_center1= [1.0,0.5,0.0, 0.0,0.0,0.0,1.0]
    elif t.robot.Is('Baxter'):
      l.o_center1= [1.1,0.5,0.0, 0.0,0.0,0.0,1.0]
    t.state_validity_checker.AddBoxToScene(l.o_center1,l.o_dim1,name='pole')

    #Attach an object to the robot:
    l.o_dim2= [0.3, 0.3, 0.5]
    #l.o_dim2= [2.3, 2.3, 2.5]
    if t.robot.Is('PR2'):
      l.lw_o_center2= [0.25,0.0,0.0, 0.0,0.0,0.0,1.0]
    elif t.robot.Is('Baxter'):
      l.lw_o_center2= [0.17,0.0,0.0, 0.0,0.0,0.0,1.0]
      l.lw_o_center2= Transform(ToList(QFromAxisAngle([0.,1.,0.],-math.pi/2.0)), l.lw_o_center2)
    t.state_validity_checker.AddBoxToRobotHand(l.lw_o_center2,l.o_dim2,arm=LEFT,name='l_grasped')

    def VizObjs(c1,c2):
      l.o_center2= Transform(t.robot.FK(arm=LEFT),l.lw_o_center2)
      t.viz.scene.AddCube(l.o_center2, l.o_dim2, rgb=t.viz.scene.ICol(c1), alpha=0.3, mid=1)
      t.viz.scene.AddCube(l.o_center1, l.o_dim1, rgb=t.viz.scene.ICol(c2), alpha=0.3, mid=0)

  elif test_case=='cyl':
    #Using Cylinder:
    #Add an object to scene:
    l.o_dim1= [1.0, 0.5*0.2]
    if t.robot.Is('PR2'):
      l.o_center1= [1.0,0.5,0.0, 0.0,0.0,0.0,1.0]
    elif t.robot.Is('Baxter'):
      l.o_center1= [1.1,0.5,0.0, 0.0,0.0,0.0,1.0]
    t.state_validity_checker.AddCylinderToScene(l.o_center1,l.o_dim1,name='pole')

    #Attach an object to the robot:
    l.o_dim2= [0.5, 0.5*0.3]
    if t.robot.Is('PR2'):
      l.lw_o_center2= [0.25,0.0,0.0, 0.0,0.0,0.0,1.0]
    elif t.robot.Is('Baxter'):
      l.lw_o_center2= [0.17,0.0,0.0, 0.0,0.0,0.0,1.0]
      l.lw_o_center2= Transform(ToList(QFromAxisAngle([0.,1.,0.],-math.pi/2.0)), l.lw_o_center2)
    t.state_validity_checker.AddCylinderToRobotHand(l.lw_o_center2,l.o_dim2,arm=LEFT,name='l_grasped')

    def VizObjs(c1,c2):
      l.o_center2= Transform(t.robot.FK(arm=LEFT),l.lw_o_center2)
      p1= Transform(l.o_center2,Vec([0.0,0.0,-0.5*l.o_dim2[0]]))
      p2= Transform(l.o_center2,Vec([0.0,0.0,0.5*l.o_dim2[0]]))
      t.viz.scene.AddCylinder(p1,p2,2.0*l.o_dim2[1], rgb=t.viz.scene.ICol(c1), alpha=0.3, mid=1)
      p1= Transform(l.o_center1,Vec([0.0,0.0,-0.5*l.o_dim1[0]]))
      p2= Transform(l.o_center1,Vec([0.0,0.0,0.5*l.o_dim1[0]]))
      t.viz.scene.AddCylinder(p1,p2,2.0*l.o_dim1[1], rgb=t.viz.scene.ICol(c2), alpha=0.3, mid=0)

  else:
    def VizObjs(c1,c2):
      pass

  VizObjs(1,2)

  if t.robot.Is('PR2'):
    CPrint(3, 'Ignoring collision between %r and %r'%('.l_gripper','l_gripper_sensor_mount_link'))
    t.state_validity_checker.IgnoreCollision('.l_gripper','l_gripper_sensor_mount_link')
  elif t.robot.Is('Baxter'):
    CPrint(3, 'Ignoring collision between %r and %r'%('display','.head'))
    t.state_validity_checker.IgnoreCollision('display','.head')

  t.state_validity_checker.SendToServer()

  l.counter= 0
  def LeftJointStateCallback():
    if l.ljscallback_init:
      l.ljscallback_start_time= rospy.Time.now()
      l.ljscallback_init= False

    q= t.robot.Q(LEFT)
    res= t.state_validity_checker.IsValidState(q,arm=LEFT)
    VisualizeContacts(res.contacts, with_normal=True, pt_size=0.03, dt=rospy.Duration(0.1))
    #print res.contacts
    if res.valid:
      VizObjs(1,2)
      print l.counter, '-'
    else:
      VizObjs(0,0)
      print l.counter, GetCollidingObjects(res.contacts)
    l.counter+= 1

  def Curve(t):
    w= 1.0
    return -0.15*math.pi*(math.sin(w*t))

  #Move the left arm with a sin curve
  curr_time= 0.0
  dt= 0.01
  if t.robot.Is('PR2'):
    base= [0.0]*7
    base[2]= math.pi*0.5
  elif t.robot.Is('Baxter'):
    base= [-math.pi/4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #base[2]= math.pi*0.5
  t.kbhit.Activate()
  waiting= False
  try:
    while curr_time<3.14159*2.0:
      if not t.kbhit.IsActive():  break
      key= t.kbhit.KBHit()
      if key=='q':
        break;
      elif key==' ':
        waiting= not waiting
      LeftJointStateCallback()

      if waiting:
        time.sleep(dt)
        continue

      curr_time= curr_time+dt
      q_traj= [copy.deepcopy(base)]
      if t.robot.Is('PR2'):
        q_traj[0][0]+= Curve(curr_time)
        q_traj[0][3]+= Curve(curr_time)
        blocking= False
      elif t.robot.Is('Baxter'):
        q_traj[0][0]+= Curve(curr_time)
        #q_traj[0][2]+= Curve(curr_time)
        blocking= True

      t.robot.FollowQTraj(q_traj, [dt], arm=LEFT, blocking=blocking)
      start_time= rospy.get_rostime()
      while rospy.get_rostime() < start_time + rospy.Duration(dt):
        time.sleep(dt*0.1)
  except Exception as e:
    PrintException(e, ' in test_col')
  finally:
    t.kbhit.Deactivate()

  Close(t)

