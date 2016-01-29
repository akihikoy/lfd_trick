#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move left arm/gripper to init posture.
  Usage: init0 [BLOCKING]
    BLOCKING: Block the process or not (default: False).
'''
def Run(t,*args):
  blocking= args[0] if len(args)>0 else False

  if t.robot.Is('PR2'):
    #angles= [0.560346004529, 0.406872786178, 1.69814343251, -1.89632475525, -28.3134341521, -1.48083122174, -4.64978978552]
    angles= [0.603789072024, 0.471672497276, 1.85064087238, -2.09393677629, -28.1654034566, -1.72261029073, -4.76443578999]
    angles= [0.48729783262306103, 0.5076493497258632, 1.8506637154437104, -2.1147837587288314, -3.0056602498891145, -1.8564307782457792, 1.4902347755205518]
  elif t.robot.Is('Baxter'):
    angles= [-0.5127330777648926, -0.7654564122802735, -0.13767477555541993, 2.5398886866394044, 0.4371845240478516, -1.3917040682189943, 0.3405437345214844]
  t.robot.OpenGripper(arm=LEFT, blocking=blocking)
  t.robot.MoveToQ(angles,dt=4.0, arm=LEFT,blocking=blocking)
