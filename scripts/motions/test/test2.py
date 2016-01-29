#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test of using other script.
  Usage: test2'''
def Run(t,*args):
  m_test= t.LoadMotion('test.test')
  print t
  m_test.Run(t, 3.14*2)
  print m_test.Help()
