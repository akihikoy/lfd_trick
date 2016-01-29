#!/usr/bin/python
#\file    test_bx.py
#\brief   Test TRobotBaxter
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Dec.08, 2015
'''
Launch before execution:
$ roslaunch lfd_trick bx_real.launch
'''
import roslib; roslib.load_manifest('lfd_trick')
from base import *

if __name__=='__main__':
  rospy.init_node('lfd_test')
  robot= TRobotBaxter()
  print 'Initializing...'
  robot.Init()
  print 'Done.'

  q0= [
    [1.0185632419921875, -0.13920875634155275, -0.24850488735351564, 1.7452866394226076, -0.08436894323730469, -0.32060198430175785, 0.01725728384399414],
    [-0.9088836157836915, -0.022626216595458985, 0.2519563441223145, 1.7023351774108888, 0.45252433190917973, -0.7366942725402833, -0.44447093278198246] ]
  x0= [
    [ 0.58030085, -0.31027952, -0.24353468, -0.15019472, 0.97193656, -0.10384798, 0.14831206],
    [ 0.58707342,  0.28459768, -0.24929951,  0.08526601,  0.9586372,  -0.05524672, 0.2658801 ] ]

  q1= [
    [0.8264321485290528, -0.21092235809326174, -0.14150972752075197, 1.7414516874572754, -0.023393206988525393, -1.3088691057678223, 0.02300971179199219],
    [-0.7731263162109375, -0.10354370306396485, 0.20555342534179688, 1.7456701346191408, 0.4513738463195801, -1.4829759249938965, -0.15148060263061525] ]
  x1= [
    [ 0.77867004, -0.30044063, -0.02990817, -0.05324578,  0.77880879, -0.04025844,  0.62369946],
    [ 0.69312312,  0.26250569, -0.07036491,  0.15024859,  0.77267414, -0.12000001,  0.60497936] ]

  def interrupt(msg):
    res= raw_input(msg)
    if res=='q':  sys.exit(0)
    
  interrupt('continue: Q > ')
  print 'Q(arm=RIGHT)=', robot.Q(arm=RIGHT)
  print 'Q(arm=LEFT)=', robot.Q(arm=LEFT)

  interrupt('continue: FK > ')
  print 'FK(arm=RIGHT)=', robot.FK(arm=RIGHT)
  print 'FK(arm=LEFT)=', robot.FK(arm=LEFT)

  interrupt('continue: IK > ')
  print 'IK(x1[RIGHT],arm=RIGHT)=', robot.IK(x1[RIGHT],arm=RIGHT)
  print '  Error=',Dist(q1[RIGHT], robot.IK(x1[RIGHT],arm=RIGHT))
  print 'IK(x1[LEFT],arm=LEFT)=', robot.IK(x1[LEFT],arm=LEFT)
  print '  Error=',Dist(q1[LEFT], robot.IK(x1[LEFT],arm=LEFT))
  
  interrupt('continue: FollowQTraj > ')
  q_traj= [q0[RIGHT],q1[RIGHT],q0[RIGHT]]
  t_traj= [3.0,6.0,9.0]
  robot.FollowQTraj(q_traj,t_traj,arm=RIGHT,blocking=True)
  q_traj= [q0[LEFT],q1[LEFT],q0[LEFT]]
  t_traj= [3.0,6.0,9.0]
  robot.FollowQTraj(q_traj,t_traj,arm=LEFT,blocking=True)
  
  interrupt('continue: FollowXTraj > ')
  x_traj= [x0[RIGHT],x1[RIGHT],x0[RIGHT]]
  t_traj= [3.0,6.0,9.0]
  robot.FollowXTraj(x_traj,t_traj,arm=RIGHT,blocking=True)
  x_traj= [x0[LEFT],x1[LEFT],x0[LEFT]]
  t_traj= [3.0,6.0,9.0]
  robot.FollowXTraj(x_traj,t_traj,arm=LEFT,blocking=True)

  interrupt('continue: MoveToQ > ')
  robot.MoveToQ(q0[RIGHT],arm=RIGHT,blocking=True)
  robot.MoveToQ(q0[LEFT],arm=LEFT,blocking=True)

  interrupt('continue: CloseGripper > ')
  robot.CloseGripper(arm=RIGHT)
  robot.CloseGripper(arm=LEFT)

  interrupt('continue: OpenGripper > ')
  robot.OpenGripper(arm=RIGHT,blocking=True)
  robot.OpenGripper(arm=LEFT,blocking=True)

  interrupt('continue: MoveGripper > ')
  robot.MoveGripper(0.07, arm=RIGHT,blocking=True)
  robot.MoveGripper(0.07, arm=LEFT,blocking=True)

  interrupt('continue: Head > ')
  robot.MoveHeadPan(-0.5)
  robot.NodHead()
  robot.MoveHeadPan(0.5)
  robot.NodHead()
  robot.MoveHeadPan(0.0)
  robot.NodHead()
