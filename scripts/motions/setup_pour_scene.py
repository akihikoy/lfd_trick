#!/usr/bin/python
from core_tool import *
def Help():
  return '''Setup pouring scene.
  Usage: setup_pour_scene BOTTLE_ID, CUP_ID [, BOTTLE_MARKER [, CUP_MARKER [, AUTO_TABLE]]]
    BOTTLE_ID: identifier of source bottle. e.g. 'b1'
    CUP_ID: identifier of receiving cup. e.g. 'c1'
    BOTTLE_MARKER: marker id of source bottle (default: 1).
    CUP_MARKER: marker id of receiving cup (default: 2).
    AUTO_TABLE: automatically put a table model (default: True).
  '''
def Run(t,*args):
  bottle= args[0]
  cup= args[1]
  m_id_bottle= args[2] if len(args)>2 else 1
  m_id_cup= args[3] if len(args)>3 else 2
  auto_table= args[4] if len(args)>4 else True

  #Registering the base markers for inferring bottle/cup poses from the AR markers
  print 'Marker observation: %i, %r' % (m_id_bottle, m_id_bottle in t.ar_markers)
  print 'Marker observation: %i, %r' % (m_id_cup, m_id_cup in t.ar_markers)
  if m_id_bottle in t.ar_markers:
    t.SetAttr(bottle,'base_marker_id',  m_id_bottle)
  if m_id_cup in t.ar_markers:
    t.SetAttr(cup,'base_marker_id',  m_id_cup)

  if auto_table:
    #Rough estimation of the table
    t.ExecuteMotion('infer', bottle,'x')
    x_b= t.GetAttr(bottle,'x')
    t.ExecuteMotion('infer', cup,'x')
    x_c= t.GetAttr(cup,'x')
    x_ts=  [[0.95,0.0,min(x_b[2],x_c[2]), 0.0,0.0,0.0,1.0],
            [0.8,0.0,min(x_b[2],x_c[2]), 0.0,0.0,0.0,1.0]]
    #table: table object.
    #table0: table object for height measurement.
    tdims= [[0.5,0.8,0.05],
            [0.15,0.2,0.02]]
    tables= ['table','table0']
    for i in (0,1):
      tdim= tdims[i]
      x_t= x_ts[i]
      table_attr={
          'x': x_t,
          'bound_box': {'dim':tdim,'center':[0.0,0.0,-tdim[2]*0.5, 0.0,0.0,0.0,1.0]},
          'shape_primitives': [
            {
              'kind': 'rtpkCuboid',
              'param': [l/2.0 for l in tdim],  #[half_len_x,half_len_y,half_len_z]
              'pose': [0.0,0.0,-tdim[2]*0.5, 0.0,0.0,0.0,1.0],
              },
            ],
        }
      t.SetAttr(tables[i],table_attr)
    #t.SetAttr('table','x',x_t)
    #t.SetAttr('table','bound_box',{'dim':tdim,'center':[0.0,0.0,0.0, 0.0,0.0,0.0,1.0]})



