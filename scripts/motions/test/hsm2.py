#!/usr/bin/python
from core_tool import *
def Help():
  return '''Hierarchical state machine test for better memory management.
  Usage: test_hsm'''
def GetN(t,*args):
  sm= TStateMachine()
  #sm.Debug= True

  sm.l= TContainer(debug=True)

  sm.l.start_time= 0
  #sm.l.myself= sm.l  #TEST: now, it is difficult to release l
  sm.l.x= TContainer()  #This does not do anything
  def GetStartTime():
    CPrint(sm.l.idx, 'l:',sm.l)
    sm.l.start_time= time.time()
  def Cleanup():
    idx= sm.l.idx
    print '--4-%i--'%idx
    del sm.l
    print '--5-%i--'%idx

  sm.l.idx= int(args[0])
  sm.StartState= 'start'
  sm['start']= TFSMState()
  sm['start'].EntryAction= lambda: CPrint(sm.l.idx, 'Hello sub SM %i!' % sm.l.idx)
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: True
  sm['start'].Actions[-1].NextState= 'count'
  sm['start'].ExitAction= lambda: (CPrint(sm.l.idx, '-->'), GetStartTime())

  sm['count']= TFSMState()
  sm['count'].EntryAction= lambda: CPrint(sm.l.idx, 'Counting...')
  sm['count'].NewAction()
  sm['count'].Actions[-1].Condition= lambda: (time.time()-sm.l.start_time)>=1
  sm['count'].Actions[-1].Action= lambda: CPrint(sm.l.idx, 'Stop @'+str(time.time()-sm.l.start_time))
  sm['count'].Actions[-1].NextState= 'stop'
  sm['count'].ElseAction.Condition= lambda: True
  sm['count'].ElseAction.Action= lambda: (CPrint(sm.l.idx, str(time.time()-sm.l.start_time)), time.sleep(0.2))
  sm['count'].ElseAction.NextState= 'count'

  sm['stop']= TFSMState()
  sm['stop'].EntryAction= lambda: CPrint(sm.l.idx, 'Finishing state machine')
  sm['stop'].ElseAction.Condition= lambda: True
  #sm['stop'].ElseAction.Action= Cleanup
  sm['stop'].ElseAction.NextState= EXIT_STATE

  print '--3-%i--'%sm.l.idx
  return sm

def Run(t,*args):
  import time

  sm= TStateMachine()
  sm.l= TContainer(debug=True)

  sm.l.subsm= 0
  def SelectSubSM():
    Print('Select sub SM:')
    sm.l.subsm= int(t.AskGen('1','2','3','0'))

  sm1= GetN(t,1)
  sm2= GetN(t,2)
  sm3= GetN(t,3)

  sm.StartState= 'start'
  sm['start']= TFSMState()
  sm['start'].EntryAction= lambda: ( Print('Hello hierarchical state machine!'), SelectSubSM() )
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: sm.l.subsm in (1,2,3)
  sm['start'].Actions[-1].NextState= 'run'
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'stop'
  sm['start'].ExitAction= lambda: Print('-->')

  sm['run']= TFSMState()
  sm['run'].NewAction()
  sm['run'].Actions[-1].Condition= lambda: sm.l.subsm==1
  sm['run'].Actions[-1].Action= lambda: sm1.Run()
  sm['run'].Actions[-1].NextState= 'start'
  sm['run'].NewAction()
  sm['run'].Actions[-1].Condition= lambda: sm.l.subsm==2
  sm['run'].Actions[-1].Action= lambda: sm2.Run()
  sm['run'].Actions[-1].NextState= 'start'
  sm['run'].NewAction()
  sm['run'].Actions[-1].Condition= lambda: sm.l.subsm==3
  sm['run'].Actions[-1].Action= lambda: sm3.Run()
  sm['run'].Actions[-1].NextState= 'start'

  sm['stop']= TFSMState()
  sm['stop'].EntryAction= lambda: Print('Finishing state machine')
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.NextState= EXIT_STATE

  sm.Run()
  print '--1--'
  sm.l= None  #NOTE: without this or sm= None, sm.l is not released
  print '--1.5--'
  #sm= None
  print '--2--'
  sm1.l= None
  sm2.l= None
  sm3.l= None
  print '--6--'

