#!/usr/bin/python
from core_tool import *
def Help():
  return '''Database operation.
  Usage:
    db [INDEXES]
    db 'show' [, INDEXES]
      Display the database.
      INDEXES: Indexes to be shown (default: None).
    db 'size'
      Show the size of the database.
    db 'clear'
      Clear the database.
    db 'load' [, DB_FILE]
      Load database from a file.
      DB_FILE: Database file in YAML format (default: DATABASE_FILE).
    db 'save' [, DB_FILE]
      Save database into a file.
      DB_FILE: Database file in YAML format (default: DATABASE_FILE).
    db 'search' COND [, MAX_COUNT]
      Search data that matches with COND from database. Return indexes.
      COND: Condition.
        e.g. lambda s,d,a: s['infer_type']=='regrab'
        where s:situation, d:inferred_data, a:assessment
      MAX_COUNT: Maximum number of searched items (default: 1).
  Note:
    The default database file DATABASE_FILE is defined in base_const.py
  '''
def Run(t,*args):
  if len(args)==0 or not isinstance(args[0],str):
    command= 'show'
  else:
    command= args[0]
    args= args[1:]

  if command=='show':
    indexes= args[0] if len(args)>0 else None
    if indexes is None:
      for situation, inferred_data, assessment in t.database:
        print CStr(1,'Situation:'),situation
        CPrint(1, '  Inferred data:')
        for keys, value in inferred_data:
          #print '  [%s]: %r' % (']['.join(keys), value)
          print '  %s[%s]%s: %r' % (ACol.I(2),']['.join(keys),ACol.I(), value)
        print CStr(1,'  Assessment:'),assessment
    else:
      for i in indexes:
        situation, inferred_data, assessment= t.database[i]
        print CStr(1,'Situation:'),situation
        CPrint(1, '  Inferred data:')
        for keys, value in inferred_data:
          #print '  [%s]: %r' % (']['.join(keys), value)
          print '  %s[%s]%s: %r' % (ACol.I(2),']['.join(keys),ACol.I(), value)
        print CStr(1,'  Assessment:'),assessment

  elif command=='size':
    print 'Database size:', len(t.database)

  elif command=='clear':
    print 'Are you sure to clear the database? (size:%i)' % len(t.database)
    if t.AskYesNo():
      t.database= []

  elif command=='load':
    file_name= args[0] if len(args)>0 else DATABASE_FILE
    if not os.path.exists(file_name):
      CPrint(4,'Database file does not exist:',file_name)
      return
    if len(t.database):
      print 'Database exists.  Do you want to load from %r?' % file_name
      if not t.AskYesNo():
        return
    t.database= LoadYAML(file_name)
    print 'Loaded database (size:%i) from: %r' % (len(t.database), file_name)

  elif command=='save':
    file_name= args[0] if len(args)>0 else DATABASE_FILE
    if os.path.exists(file_name):
      print 'File %r exists.  Do you want to overwrite?' % file_name
      if not t.AskYesNo():
        return
    SaveYAML(t.database, file_name)
    print 'Saved database into: %r' % file_name

  elif command=='search':
    condition= args[0]
    max_count= args[1] if len(args)>1 else 1
    found= []
    for idx in reversed(range(len(t.database))):
      situation, inferred_data, assessment= t.database[idx]
      if condition(situation, inferred_data, assessment):
        found.append(idx)
        if len(found)>=max_count:  return found
    return found

  else:
    raise Exception('Invalid command: %r'%command)
