#!/usr/bin/python
from core_tool import *
def Help():
  return '''Calibrate external RGB-D sensor.
  Usage: calib2 [MARKER_ID [, M_WRIST_ID [, X_SENSOR [, M_WRIST]]]]
    Execute the calibration. Follow the instruction.
    MARKER_ID: Marker ID used to calibration (default=0)
    M_WRIST_ID: Marker ID on the wrist (default=3)
    X_SENSOR: Calibrate the sensor pose (default=True)
    M_WRIST: Calibrate the marker on the wrist (default=True)'''
def Run(t,*args):
  m_id= args[0] if len(args)>0 else 0
  m_wrist_id= args[1] if len(args)>1 else 3
  calib_x_sensor= args[2] if len(args)>2 else True
  calib_m_wrist= args[3] if len(args)>3 else True

  calib_angle= -0.5*math.pi
  angle_limit= -2.00
  obs_count= 200
  total_duration= 12
  ctrl_duration= 1.0*total_duration/float(obs_count)
  obs_duration= 0.0*total_duration/float(obs_count)

  if calib_x_sensor:
    if not m_id in t.ar_markers:
      print 'Marker not observed: %i' % m_id
      return

    print 'Moveing',t.robot.ArmStr,'wrist to zero...'
    i= t.robot.Arm
    angles_trg= list(t.robot.Q(i))
    print angles_trg
    angles_trg[-1]= 0.0
    t.robot.MoveToQ(angles_trg,dt=2.0,arm=i,blocking=True)
    if angles_trg[-2]+calib_angle<angle_limit:
      calib_angle= angle_limit-angles_trg[-2]

    print 'After 0.5s,',t.robot.ArmStr,'wrist-flex joint moves %f [deg]' % (calib_angle*180.0/math.pi)
    print 'OK?'
    if not t.AskYesNo():
      #exit_proc()
      return

    t.x_sensor= []
    t.l_x_m_wrist= []

    del t.ar_markers[m_id]
    time.sleep(0.5)


    marker_data_raw= []
    gripper_data_raw= []

    angles_trg= list(t.robot.Q(i))
    angle_step= calib_angle/float(obs_count)
    #time_step= duration/float(obs_count)
    for k in range(obs_count):
      start_time= rospy.Time.now()
      while rospy.Time.now() < start_time + rospy.Duration(obs_duration):
        time.sleep(obs_duration*0.02)

      if m_id in t.ar_markers:
        marker_data_raw.append(t.ARXraw(m_id))
        gripper_data_raw.append(t.robot.FK(arm=i))  # Node: position[l_wrist_flex_link]==position[l_wrist_roll_link]

        del t.ar_markers[m_id]

      angles_trg[-2]+= angle_step
      q_traj= [angles_trg]
      t_traj= [ctrl_duration]
      t.robot.FollowQTraj(q_traj, t_traj, arm=i)
      start_time= rospy.Time.now()
      while rospy.Time.now() < start_time + rospy.Duration(ctrl_duration):
        time.sleep(ctrl_duration*0.02)

    #Remove noisy data:
    x_center, radius= CircleFit3D(marker_data_raw)
    marker_data= []
    gripper_data= []
    #n_x0= QToRot(marker_data_raw[i][3:7])[:,0]
    for i in range(len(marker_data_raw)):
      n_x0= np.array(marker_data_raw[i][0:3])-x_center
      n_x= QToRot(marker_data_raw[i][3:7])[:,0]
      ang_nx= math.acos(np.dot(n_x0,n_x)/(la.norm(n_x0)*la.norm(n_x)))
      if ang_nx<10.0/180.0*math.pi:  #Acceptable angle is 10 [deg]
        marker_data.append(marker_data_raw[i])
        gripper_data.append(gripper_data_raw[i])
        #n_x0= n_x

    print '# of data: %i (originally %i)' % (len(marker_data), len(marker_data_raw))

    x_center, radius= CircleFit3D(marker_data)

    print 'x_center,radius=',x_center,radius

    #print "radius=",radius
    #print "marker_data=",marker_data
    #print "gripper_data=",gripper_data

    x_g2m= [radius,0.0,0.0, 0,0,0,1]
    p,R= XToPosRot(gripper_data[0])
    print 'R[2,1]=',R[2,1]
    if R[2,1]>0:  # gripper ey axis is up
      x_g2m[3:]= QFromAxisAngle([1,0,0],-math.pi*0.5)
    else:  # gripper ey axis is down
      x_g2m[3:]= QFromAxisAngle([1,0,0],math.pi*0.5)
    print 'x_g2m=',x_g2m
    t.x_sensor= CalibrateSensorPose(marker_data, gripper_data, x_g2m)


  #Move for calibrating marker
  if calib_x_sensor and calib_m_wrist:
    print 'Moveing',t.robot.ArmStr,'wrist'
    #i= t.robot.Arm
    #angles_trg= list(t.robot.Q(i))
    #angles_trg[-1]= 0.5*math.pi
    #angles_trg[-2]-= 0.5*calib_angle
    #t.robot.MoveToQ(angles_trg,dt=3.0,arm=i,blocking=True)
    #time.sleep(0.1)  #Wait for marker observation
    t.ExecuteMotion('initcmw')


  #Calibrate the marker on the wrist
  if calib_m_wrist:
    t.l_x_m_wrist= []
    t.ar_adjust_m_id= m_wrist_id
    time.sleep(0.1)

    if t.ar_adjust_m_id in t.ar_markers:
      del t.ar_markers[t.ar_adjust_m_id]
      i= t.ar_adjust_arm
      time.sleep(0.1)

      print 'Calibrating the marker on the wrist'
      l_x_m_wrist_data= []
      for k in range(obs_count):
        if t.ar_adjust_m_id in t.ar_markers:
          x_wrist= t.robot.FK(arm=i)
          l_x_m_wrist= TransformLeftInv(x_wrist, t.ARX(t.ar_adjust_m_id))
          l_x_m_wrist_data.append(l_x_m_wrist)
          del t.ar_markers[t.ar_adjust_m_id]
        time.sleep(0.03)
      print '# of data: %i' % (len(l_x_m_wrist_data))
      t.l_x_m_wrist= AverageXData(l_x_m_wrist_data)
    else:
      print 'Marker on the wrist is not observed:',t.ar_adjust_m_id

  print 'Done.'
  print 'x_sensor=',t.x_sensor
  print 'l_x_m_wrist=',t.l_x_m_wrist

  #exit_proc()

