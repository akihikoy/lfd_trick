#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test pose estimation with ray tracing (3).
  Usage: test_poseest3 [BOTTLE_ID1 [, BOTTLE_ID2 [, ONCE [, USE_MARKER]]]]
    BOTTLE_ID1: Label of bottle (default: 'b53').
    BOTTLE_ID2: Label of bottle (default: None).
    ONCE: Once or continuous (default: True).
    USE_MARKER: Using a marker (No. 1, 2) to initialize bottle poses (default: True).
  '''
def Run(t,*args):
  bottle1= args[0] if len(args)>0 else 'b53'
  bottle2= args[1] if len(args)>1 else None
  once= args[2] if len(args)>2 else True
  use_marker= args[3] if len(args)>3 else True

  sensor_type= 'ext'
  #sensor_type= 'lgrip'

  #Update attribute information
  t.ExecuteMotion('objs2')
  if bottle2==None:
    t.ExecuteMotion('viz',bottle1)
  else:
    t.ExecuteMotion('viz',bottle1,bottle2)

  if use_marker:
    t.SetAttr(bottle1,'base_marker_id', 1)
    t.ExecuteMotion('infer2', {},[bottle1],'x',[],True)
    if bottle2:
      t.SetAttr(bottle2,'base_marker_id', 2)
      t.ExecuteMotion('infer2', {},[bottle2],'x',[],True)

  t.ExecuteMotion('rtposeest','clear',sensor_type)
  t.ExecuteMotion('rtposeest','create',sensor_type,'scene1',[bottle1])
  if bottle2:
    t.ExecuteMotion('rtposeest','create',sensor_type,'scene2',[bottle2])
  #if bottle2==None:
    #t.ExecuteMotion('rtposeest','create',sensor_type,'scene1',[bottle1])
  #else:
    #t.ExecuteMotion('rtposeest','create',sensor_type,'scene1',[bottle1,bottle2])

  if once:
    method1= 'lin2dxy'
    CPrint(2,'Estimating the pose of',bottle1,'with',method1)
    t.ExecuteMotion('rtposeest','once',sensor_type,bottle1,3,method1)
    if bottle2:
      method2= 'lin2dxy'
      CPrint(2,'Estimating the pose of',bottle2,'with',method2)
      t.ExecuteMotion('rtposeest','once',sensor_type,bottle2,3,method2)
  else:
    if bottle2==None:
      #method1= 'xyz'
      method1= 'lin2dxy'
      #method1= 'rot2dxy'
      #method1= 'xyzrot2dxy'
      CPrint(2,'Estimating the pose of',bottle1,'with',method1)
      t.ExecuteMotion('rtposeest','run',sensor_type,[bottle1],[method1])
    else:
      method1= 'lin2dxz'
      method2= 'lin2dxy'
      CPrint(2,'Estimating the pose of',[bottle1, bottle2],'with',[method1,method2])
      t.ExecuteMotion('rtposeest','run',sensor_type,[bottle1, bottle2],[method1,method2])

    t.kbhit.Activate()
    try:
      while True:
        if t.kbhit.IsActive():
          key= t.kbhit.KBHit()
          if key=='q':
            break;
          elif key=='.':
            x= t.GetAttr(bottle1,'x')
            qd= QFromAxisAngle([1.,0.,0.],5.0/180.0*math.pi)
            x[3:]= list(MultiplyQ(qd, x[3:]))
            t.SetAttr(bottle1,'x',x)
          elif key==',':
            x= t.GetAttr(bottle1,'x')
            qd= QFromAxisAngle([1.,0.,0.],-5.0/180.0*math.pi)
            x[3:]= list(MultiplyQ(qd, x[3:]))
            t.SetAttr(bottle1,'x',x)
        else:
          break
    except Exception as e:
      PrintException(e, ' in test_poseest3')
    finally:
      t.kbhit.Deactivate()

    t.ExecuteMotion('rtposeest','stop',sensor_type)
    #t.ExecuteMotion('rtposeest','remove',sensor_type,'scene1')
    #if bottle2:
      #t.ExecuteMotion('rtposeest','remove',sensor_type,'scene2')
