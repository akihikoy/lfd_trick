#!/usr/bin/python
from core_tool import *
def Help():
  return '''Print help of motion sctipts
  Usage: help SCRIPT1 [, SCRIPT2  [, SCRIPT3 ...]]'''
def Run(t,*args):
  if len(args)==0:
    print 'Invalid arguments.'
    print Help()
  for m in args:
    sub= t.LoadMotion(str(m))
    print '#Help of',m
    print sub.Help()
