#!/usr/bin/python
from core_tool import *
def Help():
  return '''Ttracking an AR marker.
  Usage: mtrack MARKER_ID, DURATION, X_OFFSET
    MARKER_ID: Marker ID to be tracked
    DURATION: Tracking duration
    X_OFFSET: offset pose (default=[0,0,0, 0,0,0,1])
    '''
def Run(t,*args):
  m_id= args[0]
  duration= args[1]
  x_offset= args[2] if len(args)>2 else [0,0,0, 0,0,0,1]

  time_step= 0.1
  smoothing_factor= 0.1

  if not m_id in t.ar_markers:
    print 'Marker not observed: %i' % m_id
    return

  if len(t.x_sensor)!=7:
    print 'Not calibrated.  Run calib2'
    return

  arm= t.robot.Arm
  lg_x_ext= self.t.GetAttr('wrist_%s'%LRToStrs(arm),'lx')

  start_time= rospy.Time.now()
  while rospy.Time.now() < start_time + rospy.Duration(duration):
    x_raw= t.ar_markers[m_id]
    xp= x_raw.position
    xq= x_raw.orientation
    x= [xp.x,xp.y,xp.z, xq.x,xq.y,xq.z,xq.w]
    #print 'Local',m_id,':',x
    print 'Robot',m_id,':',Transform(t.x_sensor, x)

    lsensor_x_ar= Transform(x, x_offset)
    x_ar= Transform(t.x_sensor, lsensor_x_ar)

    x_curr= t.robot.FK(x_ext=lg_x_ext, arm=arm)
    x_trg= AverageX(x_curr, x_ar, smoothing_factor)
    t.robot.MoveToXI(x_trg,time_step,lg_x_ext,inum=30, arm=arm,blocking='time')

