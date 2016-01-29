#!/usr/bin/python
from core_tool import *
def Help():
  return '''Define object properties and store them into attributes.
  Usage: objs'''
def Run(t,*args):
  #NOTE: an attribute named 'l_*' denotes a vector defined on the local frame

  with t.attr_locker:
    ##Left gripper
    #if not 'wrist_l' in t.attributes:  t.attributes['wrist_l']={}
    ##Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    #if not 'x' in t.attributes['wrist_l']: t.attributes['wrist_l']['x']= []
    ##Gripper pose
    #t.attributes['wrist_l']['lx']= [0.16,0.0,0.0, 0.0,0.0,0.0,1.0]

    ##Right gripper
    #if not 'wrist_r' in t.attributes:  t.attributes['wrist_r']={}
    ##Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    #if not 'x' in t.attributes['wrist_r']: t.attributes['wrist_r']['x']= []
    ##Gripper pose
    #t.attributes['wrist_r']['lx']= [0.16,0.0,0.0, 0.0,0.0,0.0,1.0]


    #Bottle No.1
    if not 'b1' in t.attributes:  t.attributes['b1']={}
    t.attributes['b1']['help']= 'A white soft cup.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b1']: t.attributes['b1']['x']= []
    #Gripper width before grab
    t.attributes['b1']['g_pre']= 0.08
    #Grab power (max effort):
    t.attributes['b1']['f_grab']= 15.0
    #Grab pose:
    t.attributes['b1']['l_x_grab']= [0.0, 0.0, 0.015, -0.0386798980774, 0.0474739514813, 0.0058252014884, 0.998106285144]
    #Pouring edge point:
    t.attributes['b1']['l_x_pour_e']= [0.0, -0.04, 0.11, 0.0,0.0,0.0,1.0]
    #Orientation to start pouring
    t.attributes['b1']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b1']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)
    t.attributes['b1']['l_axis_shake1']= [0,0,-1]
    t.attributes['b1']['shake_width1']= 0.04
    t.attributes['b1']['l_axis_shake2']= [0,1,0]
    t.attributes['b1']['shake_width2']= 0.02
    t.attributes['b1']['l_axis_shake3']= [1,0,0]
    t.attributes['b1']['shake_width3']= 0.02
    t.attributes['b1']['l_x_tap']= [-0.011706553252399636, 0.066821805995982878, 0.085212694401439817, 0.78612053767768464, -0.45186131636266486, -0.30451820262824753, -0.29172678191144796]
    t.attributes['b1']['trick_id_means']= [1.0,0.0,0.0]

    #TEST1-1: put a receiving container left side of the robot (change the grab pose):
    #t.attributes['b1']['l_x_grab']= Transform([0,0,0]+list(QFromAxisAngle([0,0,-1],math.pi)),t.attributes['b1']['l_x_grab'])
    #t.attributes['b1']['q_pour_max']= QFromAxisAngle([0,1,0],math.pi)
    #TEST1-2: put a receiving container left side of the robot (change the pour point):
    #t.attributes['b1']['l_x_pour_e'][1]= -t.attributes['b1']['l_x_pour_e'][1]


    #Bottle No.2
    if not 'b2' in t.attributes:  t.attributes['b2']={}
    t.attributes['b2']['help']= 'A yellow plastic pot.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b2']: t.attributes['b2']['x']= []
    #Gripper width before grab
    t.attributes['b2']['g_pre']= 0.03
    #Grab power (max effort):
    t.attributes['b2']['f_grab']= 50.0
    #Grab pose:
    t.attributes['b2']['l_x_grab']= [0.0173701175184, 0.132963892485, 0.0746058303632, -0.0496690610081, 0.0729356705803, -0.0512348563445, 0.994780559637]
    #Pouring edge point:
    t.attributes['b2']['l_x_pour_e']= [-0.0017665710111, -0.15640101956, 0.169072305583, 0.0,0.0,0.0,1.0]
    #Orientation to start pouring
    t.attributes['b2']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b2']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)
    t.attributes['b2']['trick_id_means']= [1.0,0.0,0.0]


    #Bottle No.3
    if not 'b3' in t.attributes:  t.attributes['b3']={}
    t.attributes['b3']['help']= 'A green beer (Heineken) bottle, whose mouth is modified.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b3']: t.attributes['b3']['x']= []
    #Gripper width before grab
    t.attributes['b3']['g_pre']= 0.06
    #Grab power (max effort):
    t.attributes['b3']['f_grab']= 80.0
    #Grab pose:
    t.attributes['b3']['l_x_grab']= [0.00830453390238, 0.00361607117282, 0.0827408487728, -0.0127897898305, 0.0400529652898, -0.0388682992739, 0.99835937245]
    #Pouring edge point:
    t.attributes['b3']['l_x_pour_e']= [0.00290016130429, 0.00562692930508, 0.222366269117, 0.0,0.0,0.0,1.0]
    #Orientation to start pouring
    t.attributes['b3']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b3']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)
    #t.attributes['b3']['l_axis_shake']= [0,0,-1]
    t.attributes['b3']['l_axis_shake1']= [0,0,-1]
    t.attributes['b3']['shake_width1']= 0.06
    t.attributes['b3']['l_axis_shake2']= [0,1,0]
    t.attributes['b3']['shake_width2']= 0.02
    t.attributes['b3']['l_axis_shake3']= [1,0,0]
    t.attributes['b3']['shake_width3']= 0.02
    t.attributes['b3']['trick_id_means']= [0.0,0.0,1.0]
    t.attributes['b3']['shake_axis_theta_means']= [math.pi/4.0]
    t.attributes['b3']['shake_axis_theta_std']= 0.01

    #Bottle No.4
    if not 'b4' in t.attributes:  t.attributes['b4']={}
    t.attributes['b4']['help']= 'Handmade bottle where a white cup and a transparent plastic cup are connected.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b4']: t.attributes['b4']['x']= []
    #Gripper width before grab
    t.attributes['b4']['g_pre']= 0.08
    #Grab power (max effort):
    t.attributes['b4']['f_grab']= 20.0
    #Grab pose:
    t.attributes['b4']['l_x_grab']= [0.0, 0.0, 0.14,  0, 0, 0, 1.0]
    #Pouring edge point:
    t.attributes['b4']['l_x_pour_e']= [0.0, 0.0, 0.215, 0.0,0.0,0.0,1.0]
    #Orientation to start pouring
    t.attributes['b4']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b4']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)
    #t.attributes['b4']['l_axis_shake']= [0,0,-1]
    t.attributes['b4']['l_axis_shake1']= [0,0,-1]
    t.attributes['b4']['shake_width1']= 0.06
    t.attributes['b4']['l_axis_shake2']= [0,1,0]
    t.attributes['b4']['shake_width2']= 0.02
    t.attributes['b4']['l_axis_shake3']= [1,0,0]
    t.attributes['b4']['shake_width3']= 0.02
    t.attributes['b4']['trick_id_means']= [0.0,1.0,0.0]

    #Bottle No.5
    if not 'b5' in t.attributes:  t.attributes['b5']={}
    t.attributes['b5']['help']= 'Transparent green bottle with a blue cap.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b5']: t.attributes['b5']['x']= []
    #Gripper width before grab
    t.attributes['b5']['g_pre']= 0.08
    #Grab power (max effort):
    t.attributes['b5']['f_grab']= 50.0
    #Grab pose:
    t.attributes['b5']['l_x_grab']= [0.0, 0.0, 0.085,  0, 0, 0, 1.0]
    #Pouring edge point:
    t.attributes['b5']['l_x_pour_e']= [0.0, -0.025, 0.175, 0.0,0.0,0.0,1.0]
    #Orientation to start pouring
    t.attributes['b5']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b5']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)
    #t.attributes['b5']['l_axis_shake']= [0,0,-1]
    t.attributes['b5']['l_axis_shake1']= [0,0,-1]
    t.attributes['b5']['shake_width1']= 0.06
    t.attributes['b5']['l_axis_shake2']= [0,1,0]
    t.attributes['b5']['shake_width2']= 0.02
    t.attributes['b5']['l_axis_shake3']= [1,0,0]
    t.attributes['b5']['shake_width3']= 0.02

    #Bottle No.6
    if not 'b6' in t.attributes:  t.attributes['b6']={}
    t.attributes['b6']['help']= 'Lions head beer can, which is dark red.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b6']: t.attributes['b6']['x']= []
    #Gripper width before grab
    t.attributes['b6']['g_pre']= 0.08
    #Grab power (max effort):
    t.attributes['b6']['f_grab']= 15.0
    #Grab pose:
    t.attributes['b6']['l_x_grab']= [0.0, 0.0, 0.05,  0, 0, 0, 1.0]
    #Pouring edge point:
    t.attributes['b6']['l_x_pour_e']= [0.0, -0.018, 0.12, 0.0,0.0,0.0,1.0]
    #Orientation to start pouring
    t.attributes['b6']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b6']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)
    #t.attributes['b6']['l_axis_shake']= [0,0,-1]
    t.attributes['b6']['l_axis_shake1']= [0,0,-1]
    t.attributes['b6']['shake_width1']= 0.06
    t.attributes['b6']['l_axis_shake2']= [0,1,0]
    t.attributes['b6']['shake_width2']= 0.02
    t.attributes['b6']['l_axis_shake3']= [1,0,0]
    t.attributes['b6']['shake_width3']= 0.02
    t.attributes['b6']['trick_id_means']= [1.0,0.0,0.0]

    #Bottle No.7
    if not 'b7' in t.attributes:  t.attributes['b7']={}
    t.attributes['b7']['help']= 'Transparent tall hard plastic cup with a beach illustration.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b7']: t.attributes['b7']['x']= []
    #Gripper width before grab
    t.attributes['b7']['g_pre']= 0.08
    #Grab power (max effort):
    t.attributes['b7']['f_grab']= 50.0
    #Grab pose:
    t.attributes['b7']['l_x_grab']= [0.0, 0.0, 0.05,  0, 0, 0, 1.0]
    #Pouring edge point:
    t.attributes['b7']['l_x_pour_e']= [0.0, -0.04, 0.17, 0.0,0.0,0.0,1.0]
    #Orientation to start pouring
    t.attributes['b7']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b7']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)
    #t.attributes['b7']['l_axis_shake']= [0,0,-1]
    t.attributes['b7']['l_axis_shake1']= [0,0,-1]
    t.attributes['b7']['shake_width1']= 0.06
    t.attributes['b7']['l_axis_shake2']= [0,1,0]
    t.attributes['b7']['shake_width2']= 0.02
    t.attributes['b7']['l_axis_shake3']= [1,0,0]
    t.attributes['b7']['shake_width3']= 0.02
    t.attributes['b7']['l_x_tap']= [-0.013305315314423397, 0.054026163873771474, 0.16121146749427367, -0.72013543875519936, 0.33357436784303918, 0.24582016055923492, 0.55651194023992456]
    t.attributes['b7']['trick_id_means']= [1.0,0.0,0.0]

    #Bottle No.8
    if not 'b8' in t.attributes:  t.attributes['b8']={}
    t.attributes['b8']['help']= 'Transparent plastic bottle with a blue cap.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b8']: t.attributes['b8']['x']= []
    #Gripper width before grab
    t.attributes['b8']['g_pre']= 0.08
    #Grab power (max effort):
    t.attributes['b8']['f_grab']= 15.0
    #Grab pose:
    t.attributes['b8']['l_x_grab']= [0.0, 0.0, 0.08,  0, 0, 0, 1.0]
    #Pouring edge point:
    t.attributes['b8']['l_x_pour_e']= [0.0, -0.01, 0.21,  0, 0, 0, 1.0]
    #Orientation to start pouring
    t.attributes['b8']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b8']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)

    #Bottle No.9
    if not 'b9' in t.attributes:  t.attributes['b9']={}
    t.attributes['b9']['help']= 'Lumpy transparent plastic bottle with a wider blue cap.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b9']: t.attributes['b9']['x']= []
    #Gripper width before grab
    t.attributes['b9']['g_pre']= 0.08
    #Grab power (max effort):
    t.attributes['b9']['f_grab']= 15.0
    #Grab pose:
    t.attributes['b9']['l_x_grab']= [0.0, 0.0, 0.10,  0, 0, 0, 1.0]
    #Pouring edge point:
    t.attributes['b9']['l_x_pour_e']= [0.0, -0.015, 0.19,  0, 0, 0, 1.0]
    #Orientation to start pouring
    t.attributes['b9']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b9']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)


    #Bottle No.11 (copied from b3)
    if not 'b11' in t.attributes:  t.attributes['b11']={}
    t.attributes['b11']['help']= 'Transparent JONES apple soda bottle.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b11']: t.attributes['b11']['x']= []
    #Gripper width before grab
    t.attributes['b11']['g_pre']= 0.06
    #Grab power (max effort):
    t.attributes['b11']['f_grab']= 80.0
    #Grab pose:
    t.attributes['b11']['l_x_grab']= [0.00830453390238, 0.00361607117282, 0.0827408487728, -0.0127897898305, 0.0400529652898, -0.0388682992739, 0.99835937245]
    #Pouring edge point:
    t.attributes['b11']['l_x_pour_e']= [0.00290016130429, 0.00562692930508, 0.222366269117, 0.0,0.0,0.0,1.0]
    #Orientation to start pouring
    t.attributes['b11']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b11']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)

    #Bottle No.25
    if not 'b25' in t.attributes:  t.attributes['b25']={}
    t.attributes['b25']['help']= 'Transparent Starbucks ice coffee glass bottle.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b25']: t.attributes['b25']['x']= []
    #Gripper width before grab
    t.attributes['b25']['g_pre']= 0.08
    #Grab power (max effort):
    t.attributes['b25']['f_grab']= 80.0
    #Grab pose:
    t.attributes['b25']['l_x_grab']= [0.0, 0.0, 0.08,  0, 0, 0, 1.0]
    #Pouring edge point:
    t.attributes['b25']['l_x_pour_e']= [0.0, -0.02, 0.185,  0, 0, 0, 1.0]
    #Orientation to start pouring
    t.attributes['b25']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b25']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)
    t.attributes['b25']['l_x_tap']= [0.02404184763657051, 0.047897221695018484, 0.17690179455829111, 0.6625605200090352, -0.60242791412431751, -0.31039498720516173, -0.31898137489058548]

    #Bottle No.30
    if not 'b30' in t.attributes:  t.attributes['b30']={}
    t.attributes['b30']['help']= 'Transparent chocolate milk plastic bottle with a caw illustration.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b30']: t.attributes['b30']['x']= []
    #Gripper width before grab
    t.attributes['b30']['g_pre']= 0.08
    #Grab power (max effort):
    t.attributes['b30']['f_grab']= 15.0
    #Grab pose:
    t.attributes['b30']['l_x_grab']= [0.0, 0.0, 0.08,  0, 0, 0, 1.0]
    #Pouring edge point:
    t.attributes['b30']['l_x_pour_e']= [0.0, -0.015, 0.185,  0, 0, 0, 1.0]
    #Orientation to start pouring
    t.attributes['b30']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b30']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)

    #Bottle No.40
    if not 'b40' in t.attributes:  t.attributes['b40']={}
    t.attributes['b40']['help']= 'White-transparent milk bottle of Trader Joe\'s.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b40']: t.attributes['b40']['x']= []
    #Gripper width before grab
    t.attributes['b40']['g_pre']= 0.08
    #Grab power (max effort):
    t.attributes['b40']['f_grab']= 30.0
    #Grab pose:
    t.attributes['b40']['l_x_grab']= [0.0, 0.0, 0.11,  0, 0, 0, 1.0]
    #Pouring edge point:
    t.attributes['b40']['l_x_pour_e']= [0.0, -0.015, 0.228,  0, 0, 0, 1.0]
    #Orientation to start pouring
    t.attributes['b40']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b40']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)

    #Bottle No.41
    if not 'b41' in t.attributes:  t.attributes['b41']={}
    t.attributes['b41']['help']= 'Transparent small glass salt bottle without a cap.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b41']: t.attributes['b41']['x']= []
    #Gripper width before grab
    t.attributes['b41']['g_pre']= 0.08
    #Grab power (max effort):
    t.attributes['b41']['f_grab']= 50.0
    #Grab pose:
    t.attributes['b41']['l_x_grab']= [0.0, 0.0, 0.04,  0, 0, 0, 1.0]
    #Pouring edge point:
    t.attributes['b41']['l_x_pour_e']= [0.0, -0.015, 0.108,  0, 0, 0, 1.0]
    #Orientation to start pouring
    t.attributes['b41']['q_pour_start']= QFromAxisAngle([1,0,0],30.0/180.0*math.pi)
    #Orientation where the flow is max
    t.attributes['b41']['q_pour_max']= QFromAxisAngle([1,0,0],math.pi)

    #Bottle No.42
    if not 'b42' in t.attributes:  t.attributes['b42']={}
    t.attributes['b42']['help']= 'Transparent small glass salt bottle with a mesh cap.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['b42']: t.attributes['b42']['x']= []
    #Gripper width before grab
    t.attributes['b42']['g_pre']= t.attributes['b41']['g_pre']
    #Grab power (max effort):
    t.attributes['b42']['f_grab']= t.attributes['b41']['f_grab']
    #Grab pose:
    t.attributes['b42']['l_x_grab']= t.attributes['b41']['l_x_grab']
    #Pouring edge point:
    t.attributes['b42']['l_x_pour_e']= [0.0, -0.0, 0.108,  0, 0, 0, 1.0]
    #Orientation to start pouring
    t.attributes['b42']['q_pour_start']= t.attributes['b41']['q_pour_start']
    #Orientation where the flow is max
    t.attributes['b42']['q_pour_max']= t.attributes['b41']['q_pour_max']


    #Cup No.1
    if not 'c1' in t.attributes:  t.attributes['c1']={}
    t.attributes['c1']['help']= 'A transparent soft plastic cup.'
    #Pose [x,y,z, qx,qy,qz,qw] in robot's frame
    if not 'x' in t.attributes['c1']: t.attributes['c1']['x']= []
    #Pouring location:
    #t.attributes['c1']['l_p_pour_l']= [0.0164938693088, 0.00293250989281, 0.230512294328]
    t.attributes['c1']['l_p_pour_l']= [0.0164938693088, 0.01293250989281, 0.210512294328]
    #WARNING: l_p_pour_l is DEPRECATED, which should be replaced by l_x_pour_l, where pour_start_angle is considered

  t.ExecuteMotion('attr', 'keys')

