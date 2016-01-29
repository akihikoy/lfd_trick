#!/usr/bin/python
from core_tool import *
def Help():
  return '''Common control tools.  Do not use this directly.'''
def Run(t,*args):
  print 'Error:',Help()

#Gradually approaching to a target Cartesian pose
class TApproachToX:
  def __init__(self):
    #Following variables should be assigned:
    self.core_tool= None  #Ref to TCoreTool
    self.get_x_trg= None  #Function to get the target
    self.l_x_ext= None  #Control frame
    self.arm= None  #Arm id / hand id

    #Optional:
    self.init_callback= None
    self.additional_check= None  #Function returning bool
    self.step_callback= None
    self.exit_callback= None

    #Parameters:
    self.time_step= 0.01  #Control step in sec
    self.max_speed= [0.05, 0.1]  #Max speed; 5cm/s, 0.1rad/s
    self.using_PI= False  #Using PI control after reaching the goal (to increase goal accuracy)
    self.gain_i= 1.0  #I gain
    self.pos_tol= 0.001  #Position error tolerance (only used with using_PI)
    self.rot_tol= 0.001  #Rotation error tolerance (only used with using_PI)

    #Temporary variables:
    self.x_curr= None

  def Init(self):
    #Status:
    self.reached= False
    self.state= 'reaching'
    self.status= SUCCESS_CODE

    if self.init_callback is not None:  self.init_callback()

  def Check(self):
    if not IsSuccess(self.status):  return False
    if self.reached:  return False
    if self.additional_check is not None:  return self.additional_check()
    return True

  def Step(self):
    t= self.core_tool
    dt= self.time_step
    max_dx= [self.max_speed[0]*dt, self.max_speed[1]*dt]
    x_goal= self.get_x_trg()

    if self.x_curr is None or self.state=='PI':
      x_curr= t.robot.FK(x_ext=self.l_x_ext,arm=self.arm)
    else:
      x_curr= self.x_curr
    x_diff= DiffX(x_curr, x_goal)

    if self.state=='reaching':
      scaler= max([la.norm(x_diff[:3])/max_dx[0],la.norm(x_diff[3:])/max_dx[1]])
      if scaler>1.0:
        x_trg= AddDiffX(x_curr,Vec(x_diff)/scaler)
      else:
        x_trg= x_goal
        if self.using_PI:
          self.state= 'PI'
          self.xd_sum= Vec([0.0]*6)
        else:
          self.state= 'reached'
          self.reached= True
    elif self.state=='PI':
      self.xd_sum+= Vec(x_diff)*dt
      x_trg= AddDiffX(x_goal,self.gain_i*self.xd_sum)
      #print 'DBG:',la.norm(x_diff[:3]),la.norm(x_diff[3:])
      #print '  ',la.norm(DiffX(t.robot.FK(x_ext=self.l_x_ext,arm=self.arm), x_goal)[:3])
      if la.norm(x_diff[:3])<self.pos_tol and la.norm(x_diff[3:])<self.rot_tol:
        self.reached= True
    else:
      x_trg= x_goal

    try:
      if t.robot.Is('PR2'):
        t.robot.MoveToX(x_trg,dt,self.l_x_ext,arm=self.arm,blocking=False)
      elif t.robot.Is('Baxter'):
        t.robot.MoveToX(x_trg,dt,self.l_x_ext,arm=self.arm,blocking='time')
    except Exception as e:
      PrintException(e, ' in TApproachToX')
      self.status= FailureCode(self.__class__.__name__)
      #raise e  #Forward the exception

    if IsSuccess(self.status):
      self.x_curr= x_trg

      start_time= rospy.Time.now()
      while rospy.Time.now() < start_time + rospy.Duration(dt):
        time.sleep(dt*0.02)

      if self.step_callback is not None:  self.step_callback()

  def Exit(self):
    if self.exit_callback is not None:  self.exit_callback()


#Moving a difference of a Cartesian pose
class TMoveDiffX(TApproachToX):
  def __init__(self):
    TApproachToX.__init__(self)

    #Following variables should be assigned:
    self.core_tool= None  #Ref to TCoreTool
    self.change_x= None  #Function to change current x to a target
    self.l_x_ext= None  #Control frame
    self.arm= None  #Arm id / hand id

    #Parameters inherited from TApproachToX:
    #self.time_step
    #self.max_speed

  def Init(self):
    TApproachToX.Init(self)

    t= self.core_tool
    self.x_curr= t.robot.FK(x_ext=self.l_x_ext,arm=self.arm)
    self.x_trg= self.change_x(self.x_curr)
    #Setup for TApproachToX; function to get the target
    self.get_x_trg= lambda:self.x_trg
