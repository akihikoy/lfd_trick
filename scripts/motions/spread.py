#!/usr/bin/python
from core_tool import *
def Help():
  return '''Whole spreading procedure.
  This script calls the pour script with adding the spreading specification.
  Usage: spread BOTTLE_ID, CUP_ID [, AMOUNT_TRG [, MAX_DURATION [, CONSERVATIVE]]]
    BOTTLE_ID: identifier of source bottle. e.g. 'b1'
    CUP_ID: identifier of receiving cup. e.g. 'c1'
    AMOUNT_TRG: Target amount (default=0.8)  #NOT IMPORTANT
    MAX_DURATION: Maximum duration (default=25.0)
    CONSERVATIVE: Robot becomes conservative, i.e. asking at each step (default=False)
'''
def Run(t,*args):
  bottle= args[0]
  cup= args[1]
  amount_trg= args[2] if len(args)>2 else 0.8
  max_duration= args[3] if len(args)>3 else 25.0
  conservative= args[4] if len(args)>4 else False
  spreading= True
  return t.ExecuteMotion('pour', bottle, cup, amount_trg, max_duration, conservative, spreading)

