#!/usr/bin/python
from core_tool import *
import pr2_controllers_msgs.msg as pr2c
import actionlib_msgs.msg as am
import actionlib as al
def Help():
  return '''Control the robot by keyboard interface.
  Usage: keytorso'''

#src. https://github.com/dlaz/pr2-python-minimal/blob/master/pr2_python/src/pr2_python/torso.py
class Torso(object):
  def __init__(self):
    name = 'torso_controller/position_joint_action'
    self._ac = al.SimpleActionClient(name,pr2c.SingleJointPositionAction)
    rospy.loginfo("Waiting for torso controller action server")
    self._ac.wait_for_server()
    rospy.loginfo("Torso control action client ready")
  def Move(self, pos, dt=2.0):
    goal = pr2c.SingleJointPositionGoal(position=pos,
                                        min_duration=rospy.Duration(dt),
                                        max_velocity=1)
    rospy.loginfo("Sending torso goal")
    self._ac.send_goal_and_wait(goal)
    #res = self._ac.get_result()
    #if self._ac.get_state() != am.GoalStatus.SUCCEEDED:
      #raise Exception('Error in controlling torso')
  def Up(self, pos=0.3, dt=2.0):
    self.Move(pos, dt)
  def Down(self, pos=0.01, dt=2.0):
    self.Move(pos, dt)

def Run(t,*args):
  def Close():
    t.kbhit.Deactivate()

  torso_ctrl= Torso()
  current_trg= 0.2
  torso_ctrl.Move(current_trg, 3.0)

  step= 0.01
  dt= 0.1

  t.kbhit.Activate()
  try:
    while True:
      if t.kbhit.IsActive():
        key= t.kbhit.KBHit()
        if key==None:  continue
        if key=='q':
          break;
        elif key=='w':
          current_trg+= step
          print current_trg
          torso_ctrl.Up(current_trg, dt)
        elif key=='s':
          current_trg-= step
          print current_trg
          torso_ctrl.Down(current_trg, dt)
        else:
          print 'Pressed:',key,ord(key)
      else:
        break
  except Exception as e:
    PrintException(e, ' in keytorso')
  finally:
    Close()


