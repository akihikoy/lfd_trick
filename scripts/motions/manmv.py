#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move arm manually.
  Usage: manmv'''
def Run(t,*args):
  if not t.robot.Is('PR2'):
    CPrint(4,'manmv works only with PR2.')
    return

  print '###CAUTION:',t.robot.ArmStr,'arm is relaxed'
  t.robot.Mannequin.Activate(t.robot.Arm)
  while True:
    print 'Move',t.robot.ArmStr,'arm to preferred point'
    print 'Is it OK?'
    if t.AskYesNo():
      break
  print '###CAUTION:',t.robot.ArmStr,'arm is fixed'
  t.robot.Mannequin.Deactivate(t.robot.Arm)
