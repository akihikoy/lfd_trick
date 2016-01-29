#!/usr/bin/python
from core_tool import *
def Help():
  return '''Infer attributes (ver.1, DEPRECATED).
  Usage: infer OBJ_ID, ELEM_ID [, VERBOSE]
    OBJ_ID: identifier of object. e.g. 'b1'
    ELEM_ID: element name to be inferred
    VERBOSE: if True, inferred information is printed (default:True)'''
def Run(t,*args):
  obj= args[0]
  elem= args[1]
  verbose= args[2] if len(args)>2 else True

  return t.ExecuteMotion('infer2', {},[obj],elem,[],verbose)
