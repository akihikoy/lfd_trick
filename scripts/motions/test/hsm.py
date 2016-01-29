#!/usr/bin/python
from core_tool import *
def Help():
  return '''Hierarchical state machine test.
  Usage: test_hsm'''
def GetN(t,*args):
  l= TContainer(debug=True)

  l.start_time= 0
  #l.myself= l  #TEST: now, it is difficult to release l
  l.x= TContainer()  #This does not do anything
  def GetStartTime():
    CPrint(l.idx, 'l:',l)
    l.start_time= time.time()

  l.idx= int(args[0])
  sm= TStateMachine()
  #sm.Debug= True
  sm.StartState= 'start'
  sm['start']= TFSMState()
  sm['start'].EntryAction= lambda: CPrint(l.idx, 'Hello sub SM %i!' % l.idx)
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: True
  sm['start'].Actions[-1].NextState= 'count'
  sm['start'].ExitAction= lambda: (CPrint(l.idx, '-->'), GetStartTime())

  sm['count']= TFSMState()
  sm['count'].EntryAction= lambda: CPrint(l.idx, 'Counting...')
  sm['count'].NewAction()
  sm['count'].Actions[-1].Condition= lambda: (time.time()-l.start_time)>=1
  sm['count'].Actions[-1].Action= lambda: CPrint(l.idx, 'Stop @'+str(time.time()-l.start_time))
  sm['count'].Actions[-1].NextState= 'stop'
  sm['count'].ElseAction.Condition= lambda: True
  sm['count'].ElseAction.Action= lambda: (CPrint(l.idx, str(time.time()-l.start_time)), time.sleep(0.2))
  sm['count'].ElseAction.NextState= 'count'

  sm['stop']= TFSMState()
  sm['stop'].EntryAction= lambda: CPrint(l.idx, 'Finishing state machine')
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.NextState= EXIT_STATE

  print '--3-%i--'%l.idx
  #l= None  #TEST: this will cause an error: sm['start'].EntryAction cannot access l
  return sm

def Run(t,*args):
  import time

  l= TContainer(debug=True)

  l.subsm= 0
  def SelectSubSM():
    Print('Select sub SM:')
    l.subsm= int(t.AskGen('1','2','3','0'))

  sm= TStateMachine()
  sm1= GetN(t,1)
  sm2= GetN(t,2)
  sm3= GetN(t,3)

  sm.StartState= 'start'
  sm['start']= TFSMState()
  sm['start'].EntryAction= lambda: ( Print('Hello hierarchical state machine!'), SelectSubSM() )
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: l.subsm in (1,2,3)
  sm['start'].Actions[-1].NextState= 'run'
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.NextState= 'stop'
  sm['start'].ExitAction= lambda: Print('-->')

  sm['run']= TFSMState()
  sm['run'].NewAction()
  sm['run'].Actions[-1].Condition= lambda: l.subsm==1
  sm['run'].Actions[-1].Action= lambda: sm1.Run()
  sm['run'].Actions[-1].NextState= 'start'
  sm['run'].NewAction()
  sm['run'].Actions[-1].Condition= lambda: l.subsm==2
  sm['run'].Actions[-1].Action= lambda: sm2.Run()
  sm['run'].Actions[-1].NextState= 'start'
  sm['run'].NewAction()
  sm['run'].Actions[-1].Condition= lambda: l.subsm==3
  sm['run'].Actions[-1].Action= lambda: sm3.Run()
  sm['run'].Actions[-1].NextState= 'start'

  sm['stop']= TFSMState()
  sm['stop'].EntryAction= lambda: Print('Finishing state machine')
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.NextState= EXIT_STATE

  sm.Run()
  print '--1--'
  l= None
  print '--2--'

