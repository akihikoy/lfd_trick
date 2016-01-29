#!/usr/bin/python
from core_tool import *
def Help():
  return '''Calibrate the RGB-D sensor pose from the marker on the wrist.
  Usage: m_calib_x [M_WRIST_ID]
    M_WRIST_ID: Marker ID on the wrist (default=3)'''
def Run(t,*args):
  m_wrist_id= args[0] if len(args)>0 else 3

  obs_count= 50

  t.x_sensor= []
  #Disable automatic adjustment
  t.ar_adjust_m_id= -1
  time.sleep(0.1)

  if m_wrist_id in t.ar_markers and len(t.l_x_m_wrist)==7:
    del t.ar_markers[m_wrist_id]
    i= t.ar_adjust_arm
    time.sleep(0.1)

    print 'Calibrating the sensor pose from the marker on the wrist'
    x_sensor_data= []
    for k in range(obs_count):
      if m_wrist_id in t.ar_markers:
        x_m_wrist= t.robot.FK(x_ext=t.l_x_m_wrist, arm=i)
        x_sensor= TransformRightInv(x_m_wrist,t.ARXraw(m_wrist_id))
        x_sensor_data.append(x_sensor)
        del t.ar_markers[m_wrist_id]
      time.sleep(0.03)
    print '# of data: %i' % (len(x_sensor_data))
    t.x_sensor= AverageXData(x_sensor_data)
  else:
    print 'Cannot execute the calibration'
    print '  m_wrist_id in t.ar_markers=',(m_wrist_id in t.ar_markers)
    print '  m_wrist_id=',m_wrist_id
    print '  t.l_x_m_wrist=',t.l_x_m_wrist

  #Enable automatic adjustment
  t.ar_adjust_m_id= m_wrist_id
