#!/usr/bin/python
from core_tool import *
def Help():
  return '''Throwing a paper plane with the right arm.
  Usage: throw1'''
def Run(t,*args):
  arm= RIGHT
  #q_traj= [
    #[-0.5882816314819337, 0.9038981782287598, 1.555456517138672, 2.2645391355285645, -2.2867818569274903, -1.091043834136963, -0.7612379651184082],
    #[-0.12923788123168947, 0.7313253397888184, 1.893315785284424, 1.6892963407287598, -1.9650293870361328, -0.9196214812866211, -0.6446554253723145],
    #[0.09242234236450196, 0.5863641554992676, 1.970781814984131, 1.2935292979064943, -1.9209274394348146, -0.796519523199463, -0.6845389258117677],
    #]
  #t_traj= [1.0,1.5,1.8]
  q_traj= [
    [0.7267233974304199, 1.0526943144836427, 3.0192576823059083, 2.4121847861938477, -3.039199432525635, -0.9015972070495606, -0.22894663233032228],
    [0.7995874847717286, 0.9468496402404786, 3.054539240386963, 1.6881458551391604, -3.039199432525635, -0.45482530308837893, -0.17640779040527346],
    [0.8018884559509278, 1.0009224629516602, 3.054539240386963, 1.648262354699707, -3.039582927722168, -0.2707476087524414, -0.19941750219726564],
    ]
  t_traj= [1.0,1.5,1.6]
  #LimitQTrajVel(q_start=q_traj[0], q_traj=q_traj, t_traj=t_traj, qvel_limits=t.robot.JointVelLimits(arm))

  t.robot.MoveGripper(0.01, max_effort=100.0, arm=arm)
  t.robot.MoveToQ(q_traj[0],arm=arm)
  CPrint(1,'Hold a paper plane.  Can I close the gripper?')
  if not t.AskYesNo():  return

  t.robot.MoveGripper(0.00, max_effort=100.0, speed=100.0, arm=arm)
  CPrint(1,'Follow the trajectory.  Ready?')
  if not t.AskYesNo():  return
  t.robot.FollowQTraj(q_traj, t_traj, arm=arm, blocking=False)
  #t.robot.FollowXTraj(x_traj, t_traj, arm=arm, blocking=True)

  time.sleep(1.4)
  t.robot.MoveGripper(0.03, max_effort=100.0, speed=100.0, arm=arm)
