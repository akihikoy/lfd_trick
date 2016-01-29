#!/usr/bin/python
from core_tool import *
def Help():
  return '''Template of script.
  Usage: template'''
def Run(t,*args):
  l= TContainer(debug=True)
  l.a= 100
  l.c= TContainer(debug=True)
  l.c.a= 2.5
  def Clear():
    #Test()
    CPrint(2,'l= ',l)
    CPrint(2,'l.a= ',l.a)
    CPrint(2,'l.c.a= ',l.c.a)
    CPrint(2,'l.c= ',l.c)
    l.c= None
    #l= None
  def Test():
    CPrint(2,'l= ',l)
    CPrint(2,'l.a= ',l.a)
    CPrint(2,'l.c.a= ',l.c.a)
  Test()
  CPrint(1,'1----')
  Clear()
  CPrint(1,'2----')
  l= None
  CPrint(1,'3----')
