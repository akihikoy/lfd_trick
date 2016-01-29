#!/usr/bin/python
from core_tool import *
def Help():
  return '''State machine with PAA test.
  Usage: test_sm2'''
def Run(t,*args):
  import time

  l= TContainer(debug=True)

  l.start_time= 0
  def GetStartTime():
    l.start_time= int(time.time())

  sm= TStateMachine()

  param_kind= 3
  if param_kind==1:
    sm.Params['wait_time']= TDiscParam()
    wait_time= sm.Params['wait_time']
    wait_time.Candidates= [[1.0],[1.2],[1.4],[1.6],[1.8],[2.0]]
  elif param_kind==2:
    sm.Params['wait_time']= TContParamGrad()
    wait_time= sm.Params['wait_time']
    wait_time.Mean= [1.2]
    wait_time.Min= [1.0]
    wait_time.Max= [2.0]
    #argv[0]: displacement from the target time
    wait_time.Gradient= lambda argv: [argv[0]]
  elif param_kind==3:
    sm.Params['wait_time']= TContParamNoGrad()
    wait_time= sm.Params['wait_time']
    wait_time.Mean= [1.2]
    wait_time.Std= 0.25
    wait_time.Min= [1.0]
    wait_time.Max= [2.0]
  wait_time_internal_data= None
  #wait_time.Init(wait_time_internal_data)

  sm.StartState= 'start'
  sm['start']= TFSMState()
  sm['start'].EntryAction= lambda: Print('Hello state machine!')
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: True # Print("Want to move?") or t.AskYesNo()
  sm['start'].Actions[-1].NextState= 'count'
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.Action= lambda: Print('Keep to stay in start\n')
  sm['start'].ElseAction.NextState= 'start'
  sm['start'].ExitAction= lambda: (Print('-->'), GetStartTime())

  sm['count']= TFSMState()
  sm['count'].EntryAction= lambda: ( Print('Counting...'), wait_time.Select() )
  sm['count'].NewAction()
  sm['count'].Actions[-1].Condition= lambda: (time.time()-l.start_time)>=wait_time.Param()[0]
  sm['count'].Actions[-1].Action= lambda: Print('Stop @'+str(time.time()-l.start_time))
  sm['count'].Actions[-1].NextState= 'stop'
  sm['count'].ElseAction.Condition= lambda: True
  sm['count'].ElseAction.Action= lambda: (Print(str(time.time()-l.start_time)), time.sleep(0.2))
  sm['count'].ElseAction.NextState= 'count'

  sm['stop']= TFSMState()
  if isinstance(wait_time,TDiscParam) or isinstance(wait_time,TContParamNoGrad):
    sm['stop'].EntryAction= lambda: ( Print('Finishing state machine'), wait_time.Update(1.0-abs(time.time()-l.start_time-1.6)**2 if abs(time.time()-l.start_time-1.6)<0.2 else 0.0) )
  elif isinstance(wait_time,TContParamGrad):
    sm['stop'].EntryAction= lambda: ( Print('Finishing state machine'), wait_time.Update([1.6-(time.time()-l.start_time)]) )
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.NextState= EXIT_STATE

  #sm.Run()

  for i in range(40):
    wait_time.Init(wait_time_internal_data)
    #sm.Reset()
    sm.Run()
    if isinstance(wait_time,TDiscParam):
      print wait_time.Means
      print wait_time.UCB()
    elif isinstance(wait_time,TContParamGrad):
      print wait_time.Mean
    elif isinstance(wait_time,TContParamNoGrad):
      print wait_time.Param()
    wait_time_internal_data= wait_time.Save()

  print 'wait_time_internal_data:',wait_time_internal_data

  l= None
