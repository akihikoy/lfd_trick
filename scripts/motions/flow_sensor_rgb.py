#!/usr/bin/python
from core_tool import *
def Help():
  return '''Flow sensor using an RGB camera.
  Usage: flow_sensor_rgb'''

class TFlowSensorRGB:
  #t is an instance of TCoreTool.  If t is None, you should run a custom activation/deactivation.
  def __init__(self, t):
    self.t= t  #TCoreTool
    self.callback_a_running= False
    self.callback_f_running= False
    self.sub_callback_a= None
    self.sub_callback_f= None
    self.update_interval= 3.0
    self.flow_amount_factor= 0.015  #From experience
    self.alpha_factor= 0.01
    self.var_locker= threading.RLock()
    self.is_active= False
  def __del__(self):
    self.Deactivate()
    print 'Count of var_locker:',self.var_locker._RLock__count
    print 'Deleted TFlowSensorRGB',hex(id(self))

  def Activate(self):
    self.is_active= False
    self.observed_a= False
    self.observed_f= False
    self.amount= 0.0
    self.flow= 0.0
    #self.flow_detected= False  #DEPRECATED
    self.log_flow= []
    self.seq_amount= []
    self.seq_flow_sum= []
    self.t_init= rospy.Time.now()
    if self.t!=None:
      if not self.t.ExecuteMotion('cv', 'setup'):
        CPrint(4,'cv setup failed.')
        return False
      self.t.ExecuteMotion('cv', 'resume')
      self.t.callback.amount_observer= self.CallbackA
      self.t.callback.flow_speed_observer= self.CallbackF
    self.is_active= True
    return True

  def Deactivate(self, wait_join=True):
    if not self.is_active:  return
    self.sub_callback_a= None
    self.sub_callback_f= None
    if self.t!=None:
      self.t.callback.amount_observer= None
      self.t.callback.flow_speed_observer= None
    if wait_join:
      while self.callback_a_running:  print 'A'; time.sleep(0.01)
      while self.callback_f_running:  print 'F'; time.sleep(0.01)
    if self.t!=None:
      self.t.ExecuteMotion('cv', 'pause')
    self.is_active= False

  #If time_window is zero, return if one of the last two flow is positive.
  #Else, return if the number of positive flow in time_window(sec) is
  #  bigger than flow_ratio(ratio) or not.
  #  time_window should be less than update_interval.
  def IsFlowDetected(self, time_window=0.4, flow_ratio=0.6):
    #if self.flow_detected:
      #self.flow_detected= False
      #return True
    if len(self.log_flow)==0:  return False
    if time_window<=0.0:
      if len(self.log_flow)==1:  return self.log_flow[-1][1]>0.0
      return self.log_flow[-1][1]>0.0 or self.log_flow[-2][1]>0.0
    else:
      all_count= 0
      pos_count= 0
      t_curr= rospy.Time.now()
      while all_count<len(self.log_flow) and (t_curr-self.log_flow[-(all_count+1)][0]).to_sec()<time_window:
        if self.log_flow[-(all_count+1)][1]>0.0:
          pos_count+= 1
        all_count+= 1
      return float(pos_count)/float(all_count)

  def ObservedA(self):
    with self.var_locker:  res= self.observed_a
    return res

  def ObservedF(self):
    with self.var_locker:  res= self.observed_f
    return res

  def Amount(self):
    with self.var_locker:  res= self.amount
    return res

  def FlowA(self):
    with self.var_locker:  res= self.flow_amount_factor * self.flow
    return res

  def FilteredFlowA(self, filter_size=5):
    if len(self.seq_flow_sum)==0:
      return self.FlowA()
    elif len(self.seq_flow_sum)==1:
      f= self.seq_flow_sum[-1][1] / (self.seq_flow_sum[-1][0]-self.t_init).to_sec()
    else:
      fsize= min(len(self.seq_flow_sum), filter_size)
      f= (self.seq_flow_sum[-1][1]-self.seq_flow_sum[-fsize][1]) / (self.seq_flow_sum[-1][0]-self.seq_flow_sum[-fsize][0]).to_sec()
    return self.flow_amount_factor*f

  def UpdateA(self):
    t_curr= rospy.Time.now()
    self.seq_amount.append([t_curr,self.amount])
    if (t_curr-self.seq_amount[0][0]).to_sec() > self.update_interval:
      self.seq_amount.pop(0)
    self.UpdateModel()

  def UpdateF(self):
    t_curr= rospy.Time.now()
    self.log_flow.append([t_curr,self.flow])
    while (t_curr-self.log_flow[0][0]).to_sec() > self.update_interval:
      self.log_flow.pop(0)

    if len(self.seq_flow_sum)==0:
      F= self.flow*(t_curr-self.t_init).to_sec()
    else:
      F= self.seq_flow_sum[-1][1]
      F+= self.flow*(t_curr-self.seq_flow_sum[-1][0]).to_sec()
    self.seq_flow_sum.append([t_curr,F])
    while (t_curr-self.seq_flow_sum[0][0]).to_sec() > self.update_interval:
      self.seq_flow_sum.pop(0)
    #self.UpdateModel()

  def UpdateModel(self):
    if len(self.seq_amount)>=2 and len(self.seq_flow_sum)>=2:
      amount_diff= self.seq_amount[-1][1]-self.seq_amount[0][1]
      flow_sum_diff= self.seq_flow_sum[-1][1] - self.seq_flow_sum[0][1]
      if amount_diff>0.1 and flow_sum_diff>1.0e-6:
        factor= amount_diff / flow_sum_diff
        self.flow_amount_factor= (1.0-self.alpha_factor)*self.flow_amount_factor + self.alpha_factor*factor

  def CallbackA(self,msg):
    with self.var_locker:
      self.callback_a_running= True
      self.amount= msg.data
      try:
        self.UpdateA()
        if self.sub_callback_a<>None:  self.sub_callback_a(msg)
        self.observed_a= True
      except Exception as e:
        PrintException(e)
        self.Deactivate(wait_join=False)
      finally:
        self.callback_a_running= False

  def CallbackF(self,msg):
    with self.var_locker:
      self.callback_f_running= True
      #TEST:flow condition should be considered again
      #msg.data[0]: flow speed, msg.data[1]: flow angle
      #if msg.data[1]>0.5*math.pi-0.2 and msg.data[1]<0.5*math.pi+0.2 and msg.data[0]>20.0:
      if msg.data[0]>2.0:  #TEST:FIXME
        self.flow= msg.data[0]/10.0
        #self.flow_detected= True
      else:
        self.flow= 0.0
      try:
        self.UpdateF()
        if self.sub_callback_f<>None:  self.sub_callback_f(msg)
        self.observed_f= True
      except Exception as e:
        PrintException(e)
        self.Deactivate(wait_join=False)
      finally:
        self.callback_f_running= False

class TFlowAmountModel (TFlowSensorRGB):
  def __init__(self, t):
    TFlowSensorRGB.__init__(self, t)
    #Initial amount of source container and receiving container
    self.amount0_src= 1.0
    self.amount0_rcv= 0.0
    self.amount0_initialized= False
    #Model parameters of theta0 where the flow starts
    self.th0_p1= -0.2
    self.th0_p2= 1.6
    #Parameter update ratio:
    self.alpha_amount0= 0.2
    self.alpha_th0_p= 0.1
  def __del__(self):
    TFlowSensorRGB.__del__(self)

  #def Activate(self):
    #self.amount0_initialized= ...
    #self.amount0_src= ...
    #TFlowSensorRGB.Activate(self)

  #def Deactivate(self, wait_join=True):
    #TFlowSensorRGB.Deactivate(self, wait_join)
    #with self.var_locker:
      #self.amount0_src= self.amount0_src - self.Amount()

  #Estimate amount in the source container
  def AmountSrc(self):
    damount_src= -(self.Amount()-self.amount0_rcv)  #Increase of rcv is decrease of src
    amount_src= self.amount0_src+damount_src
    return amount_src

  #Estimate theta0 where the flow starts
  def Theta0(self, amount_src=None):
    if amount_src==None:  amount_src= self.AmountSrc()
    return self.th0_p1*amount_src + self.th0_p2

  #Execute this function to update the model at an initial flow is observed
  def UpdateAtInitialFlow(self, theta0):
    if not self.amount0_initialized:
      self.amount0_src= (theta0-self.th0_p2)/self.th0_p1
      self.amount0_rcv= self.Amount()
      self.amount0_initialized= True
    else:
      amount_src= self.AmountSrc()
      err= theta0 - self.Theta0(amount_src)  #Error of theta0 estimation
      #Updating state (initial amount_src) and model parameters with simplified Newton Raphson
      self.amount0_src+= self.alpha_amount0*err/self.th0_p1
      self.th0_p1+= self.alpha_th0_p*err/amount_src
      self.th0_p2+= self.alpha_th0_p*err

def LogFlow(flow_sensor, logfp):
  if not flow_sensor.ObservedA() or not flow_sensor.ObservedF():
    print 'Observation:',flow_sensor.ObservedA(),flow_sensor.ObservedF()
    return
  damount= 0.0
  if len(flow_sensor.seq_amount)==1:
    damount= flow_sensor.seq_amount[-1][1]/(flow_sensor.seq_amount[-1][0]-flow_sensor.t_init).to_sec()
  elif len(flow_sensor.seq_amount)>1:
    damount= (flow_sensor.seq_amount[-1][1]-flow_sensor.seq_amount[-2][1])/(flow_sensor.seq_amount[-1][0]-flow_sensor.seq_amount[-2][0]).to_sec()
  line= '%f %f %f %f %f %f %f %r %r\n' % (
              flow_sensor.Amount(),
              flow_sensor.flow,
              flow_sensor.seq_flow_sum[-1][1],
              damount,
              flow_sensor.FlowA(),
              flow_sensor.FilteredFlowA(filter_size=5),
              flow_sensor.flow_amount_factor,
              flow_sensor.IsFlowDetected(0.0),
              flow_sensor.IsFlowDetected(0.2) )
  print line,
  logfp.write(line)

def Run(t,*args):
  #logfp= file(t.LogFileName('flow_rgb_'),'w')
  logfp= file('/dev/stdout','w')
  flow_sensor= TFlowSensorRGB(t)
  flow_sensor.flow_amount_factor= 0.013884
  #flow_sensor.flow_amount_factor= 0.0035  #NOTE: Good for ode_pour_sim
  flow_sensor.sub_callback_a= lambda m: LogFlow(flow_sensor, logfp)
  #flow_sensor.sub_callback_f= lambda m: Print(flow_sensor.amount, flow_sensor.flow)
  CPrint(3,'Start after 2 sec...')
  #time.sleep(2)
  flow_sensor.Activate()
  time.sleep(5)
  #time.sleep(5)
  #for i in range(200):
    #if flow_sensor.IsFlowDetected():
      #print 'Flow detected'
      #break
    #time.sleep(0.1)
  print 'flow_sensor.flow_amount_factor=',flow_sensor.flow_amount_factor
  #print flow_sensor.log_flow
  flow_sensor.Deactivate()
