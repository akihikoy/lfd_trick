#!/usr/bin/python
from core_tool import *
def Help():
  return '''Control the robot by keyboard interface.
  Usage: keyctrl'''

def MovePD(t,x0,pd):
  dt= 0.01
  x_trg= [0.0]*7
  x_trg[:3]= [x0[d]+pd[d] for d in range(3)]
  x_trg[3:]= x0[3:]
  print 'x_trg=',x_trg
  #t.robot.MoveToX(x_trg,dt)
  angles= t.robot.IK(x_trg)
  if angles is not None:
    t.robot.FollowQTraj([angles],[dt],blocking=True)
    #start_time= rospy.Time.now()
    #while rospy.Time.now() < start_time + rospy.Duration(dt):
      #time.sleep(dt*0.02)
    x0[:3]= x_trg[:3]
  else:
    CPrint(4,'IK error')

def Run(t,*args):
  x0= t.robot.FK()

  def Close():
    t.kbhit.Deactivate()
    t.ExecuteMotion('scene', 'clear')

  step= 0.005

  t.kbhit.Activate()
  t.ExecuteMotion('scene', 'make')
  try:
    while True:
      if t.kbhit.IsActive():
        key= t.kbhit.KBHit()
        if key==None:  continue
        if key=='q':
          break;
        elif key=='w':
          MovePD(t,x0,[-step,0.0,0.0])
        elif key=='s':
          MovePD(t,x0,[+step,0.0,0.0])
        elif key=='d':
          MovePD(t,x0,[0.0,+step,0.0])
        elif key=='a':
          MovePD(t,x0,[0.0,-step,0.0])
        elif key=='x':
          MovePD(t,x0,[0.0,0.0,+step])
        elif key=='z':
          MovePD(t,x0,[0.0,0.0,-step])
        else:
          print 'Pressed:',key,ord(key)
      else:
        break
      res= t.ExecuteMotion('scene','isvalidq',t.robot.Arm,[],t.robot.Q(),True)
      if not res[0]:
        print 'Collision:',GetCollidingObjects(res[1].contacts)
      else:
        print '---'
  except Exception as e:
    PrintException(e, ' in keyctrl')
  finally:
    Close()

