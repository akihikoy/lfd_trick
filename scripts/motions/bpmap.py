#!/usr/bin/python
from core_tool import *
def Help():
  return '''Assign a base point (bp) pose to an object's pose (x) [DEPRECATED].
  Usage: bpmap BP_ID1, OBJ_ID1 [, BP_ID2, OBJ_ID2 [, BP_ID3, OBJ_ID3 ...]]'''
def Run(t,*args):
  if len(args)<2 or len(args)%2!=0:
    print Help()
    return
  with t.attr_locker:
    for i in range(len(args)/2):
      if not args[2*i] in t.base_x:
        print 'No base point named:',args[2*i]
        return
      if not args[2*i+1] in t.attributes:
        print 'No object named:',args[2*i+1]
        return
      t.attributes[args[2*i+1]]['x']= t.base_x[args[2*i]]
      print "["+args[2*i+1]+"]['x']=",t.attributes[args[2*i+1]]['x']
