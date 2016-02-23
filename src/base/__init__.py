import os
#using_ros= ('ros' in ' '.join(sys.path))
using_ros= True if 'ROS_ROOT' in os.environ else False

#__all__= ['base_adv',...,'ros_viz']
__BASE_NONROS__= [
  #'base_chn',  #Internal use only.
  'base_const',
  'base_exp',
  'base_ml',
  #'base_ml_dnn',
  #'base_ml_lwr',  #DEPRECATED
  'base_ml_lwr2',
  'base_opt',
  'base_opt2',
  'base_sm',
  'base_sys',
  'base_util',
  ]
__BASE_ROS__= [
  'base_adv',
  'base_geom',
  'base_traj',
  'ros_base',
  #'ros_col',  #DEPRECATED
  'ros_col_mi',
  'ros_robot',
  'ros_viz',
  ]
__BASE_ROS_RBT__= [
  'ros_rbt_bxtr',
  'ros_rbt_pr2',
  ]
__BASE_ALL__= (__BASE_NONROS__ if not using_ros else (__BASE_NONROS__ + __BASE_ROS__ + __BASE_ROS_RBT__))

try:
  __BASE_RELOADED__
except NameError:
  __BASE_RELOADED__= False

if __BASE_RELOADED__:
  for m in __BASE_ALL__:
    print 'reloading',m
    if m not in __BASE_ROS_RBT__:
      sub=__import__(m,globals(),locals(),m,-1)
      reload(sub)
    else:
      try:
        sub=__import__(m,globals(),locals(),m,-1)
        reload(sub)
      except ImportError as e:  print str(e)
else:
  __BASE_RELOADED__= True


#from base_chn import *  #Internal use only.
from base_const import *
from base_exp import *
from base_ml import *
#from base_ml_dnn import *
#from base_ml_lwr import *
from base_ml_lwr2 import *
from base_opt import *
from base_opt2 import *
from base_sm import *
from base_sys import *
from base_util import *

if using_ros:
  from base_adv import *
  from base_geom import *
  from base_traj import *
  from ros_base import *
  #from ros_col import *
  from ros_col_mi import *
  from ros_robot import *
  from ros_viz import *

  try:  from ros_rbt_bxtr import *
  except ImportError as e:  print str(e)
  try:  from ros_rbt_pr2 import *
  except ImportError as e:  print str(e)
