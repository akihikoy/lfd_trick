#! /usr/bin/env python
#Basic tools (constant values).
import os

USING_ROS= True if 'ROS_ROOT' in os.environ else False
ROS_ROBOT= os.environ['ROS_ROBOT'] if 'ROS_ROBOT' in os.environ else None
ROS_DISTRO= os.environ['ROS_DISTRO'] if 'ROS_DISTRO' in os.environ else None

if ROS_ROBOT in ('PR2','PR2_SIM'):
  ROS_DEFAULT_FRAME= 'torso_lift_link'
elif ROS_ROBOT in ('Baxter','Baxter_SIM'):
  ROS_DEFAULT_FRAME= 'torso'
  #ROS_DEFAULT_FRAME= 'base'
else:
  ROS_DEFAULT_FRAME= 'base'


#Arm/gripper ID
RIGHT=0
LEFT=1

#Attribute key for the current (temporary) situation
CURR='*'

#Database structure:
#database= [situation, inferred_data, assessment] * N1
#situation= {dict}
#inferred_data= [keys, value] * N2
#assessment= {dict}
DB_SITUATION= 0
DB_INFERRED= 1
DB_ASSESSMENT= 2
DB_INFERRED_KEYS= 0
DB_INFERRED_VALUE= 1

#Database file name
DATABASE_FILE= '.database.yaml'

#Memory file name
MEMORY_FILE= '.memory.yaml'

def LRToStr(whicharm):
  if whicharm==RIGHT: return 'Right'
  if whicharm==LEFT:  return 'Left'
  return None

def LRTostr(whicharm):
  if whicharm==RIGHT: return 'right'
  if whicharm==LEFT:  return 'left'
  return None

def LRToStrS(whicharm):
  if whicharm==RIGHT: return 'R'
  if whicharm==LEFT:  return 'L'
  return None

def LRToStrs(whicharm):
  if whicharm==RIGHT: return 'r'
  if whicharm==LEFT:  return 'l'
  return None

def StrToLR(whicharm_str):
  if whicharm_str in ('r','R','right','Right','RIGHT'):  return RIGHT
  if whicharm_str in ('l','L','left','Left','LEFT'):  return LEFT
  return None

