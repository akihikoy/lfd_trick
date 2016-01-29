#!/usr/bin/python
from core_tool import *
def Help():
  return '''Moving to an initial pose when a task has exited in failure.
    Every motion is executed interactively.
  Usage: errtoinit'''
def Run(t,*args):
  if t.GetAttr(CURR,'task')=='pour':
    if (CPrint(2,'Run "postpour"?'), t.AskYesNo())[-1]:  t.ExecuteMotion('postpour')
    if (CPrint(2,'Run "release?"'), t.AskYesNo())[-1]:  t.ExecuteMotion('release')
    if (CPrint(2,'Run "postgrab"?'), t.AskYesNo())[-1]:  t.ExecuteMotion('postgrab')
    if (CPrint(2,'Run "init0"?'), t.AskYesNo())[-1]:  t.ExecuteMotion('init0')
