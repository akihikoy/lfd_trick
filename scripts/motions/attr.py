#!/usr/bin/python
from core_tool import *
def Help():
  return '''List up the attributes or assign value to an element.
  Usage:
    attr
    attr 'list' [MAX_LEVEL]
      List up the attributes
      MAX_LEVEL: Maximum level of printed attribute
    attr 'keys' [MAX_LEVEL, ] [KEY1 [, KEY2 [, ...]]]
      List up the keys of attributes with their types (not values)
      MAX_LEVEL: Maximum level of printed attribute
      KEY*: List of attribute keys
    attr 'show' [KEY1 [, KEY2 [, ...]]]
      Show the value of the specified attribute
      KEY*: List of attribute keys
    attr 'set' KEY1 [, KEY2 [, ...]], VALUE
      Assign VALUE to the specified attribute
      KEY*: List of attribute keys
      VALUE: Value
    attr 'del' KEY1 [, KEY2 [, ...]]
      Delete the specified attribute
      KEY*: List of attribute keys
    attr 'savemem' [, MEM_FILE]
      Save 'memory' into a file.
      MEM_FILE: Memory file in YAML format (default: MEMORY_FILE).
    attr 'loadmem' [, MEM_FILE]
      Load 'memory' from a file.
      MEM_FILE: Memory file in YAML format (default: MEMORY_FILE).
    attr 'dump', DUMP_FILE
      Save the whole attributes into DUMP_FILE.
      DUMP_FILE: Data file in YAML format.
    attr 'load', YAML_FILE
      Load from YAML_FILE.
      YAML_FILE: Data file in YAML format.
  Note:
    The default memory file MEMORY_FILE is defined in base_const.py
  Example:
    attr set 'c1','x', [0,0,0, 0,0,0,1]
  '''
def Run(t,*args):
  col= 1
  c1,c2= ACol.X2(col)
  with t.attr_locker:
    if len(args)==0:
      PrintDict(t.attributes,col=col)
    else:
      command= args[0]
      args= args[1:]
      if command=='list':
        if len(args)==0:    PrintDict(t.attributes, col=col)
        elif len(args)==1:  PrintDict(t.attributes,max_level=args[0], col=col)
        else:  raise Exception('Invalid arguments for %r: %r'%(command,args))
      elif command=='keys':
        if len(args)==0:
          PrintDict(t.attributes,keyonly=True, col=col)
        else:
          if isinstance(args[0],int):
            max_level= args[0]
            keys= args[1:]
          else:
            max_level= -1
            keys= args
          value= t.GetAttr(*keys)
          if isinstance(value,dict):
            if len(keys)==0:
              PrintDict(value,max_level=max_level,level=0,keyonly=True, col=col)
            else:
              print '%s[%s]%s= ...' % (c1,']['.join(keys),c2)
              PrintDict(value,max_level=max_level,level=1,keyonly=True, col=col)
          else:
            print '%s[%s]%s= %s' % (c1,']['.join(keys),c2, type(value))
      elif command=='show':
        value= t.GetAttr(*args)
        if isinstance(value,dict):
          if len(args)==0:
            PrintDict(value,level=0, col=col)
          else:
            print '%s[%s]%s= ...' % (c1,']['.join(args),c2)
            PrintDict(value,level=1, col=col)
        else:
          print '%s[%s]%s= %r' % (c1,']['.join(args),c2, value)
      elif command=='set':
        t.SetAttr(*args)
        print 'Set:'
        Run(t,'show',*(args[:-1]))
      elif command=='del':
        t.DelAttr(*args)
        print 'Deleted: [%s]' % (']['.join(args))
      elif command=='savemem':
        file_name= args[0] if len(args)>0 else MEMORY_FILE
        if os.path.exists(file_name):
          print 'File %r exists.  Do you want to overwrite?' % file_name
          if not t.AskYesNo():
            return
        SaveYAML(t.GetAttrOr({},'memory'), file_name)
        print 'Saved memory into: %r' % file_name
      elif command=='loadmem':
        file_name= args[0] if len(args)>0 else MEMORY_FILE
        if not os.path.exists(file_name):
          CPrint(4,'Memory file does not exist:',file_name)
          return
        if t.HasAttr('memory'):
          print 'Memory exists.  Do you want to load from %r?' % file_name
          if not t.AskYesNo():
            return
        t.AddDictAttr('memory', LoadYAML(file_name))
        print 'Loaded memory from: %r' % (file_name)
      elif command=='dump':
        file_name= args[0]
        if os.path.exists(file_name):
          print 'File %r exists.  Do you want to overwrite?' % file_name
          if not t.AskYesNo():
            return
        SaveYAML(t.GetAttrOr({}), file_name)
        print 'Saved attributes into: %r' % file_name
      elif command=='load':
        file_name= args[0]
        print 'Do you want to load attributes from %r?' % file_name
        if not t.AskYesNo():
          return
        t.AddDictAttr(LoadYAML(file_name))
        print 'Loaded attributes from: %r' % (file_name)
      else:
        print 'Invalid command'
        print Help()
