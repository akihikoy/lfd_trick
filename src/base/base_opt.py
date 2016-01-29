#! /usr/bin/env python
#Basic tools (optimization).
import math
import random
import copy
import os
import time
import cma
from base_const import *
from base_util import *
from base_ml import *

#Assessment model
class TAssessment:
  #Timing to check the condition (more variation is possible):
  STATE_CALLBACK= int('000001',2)
  TIMER_CALLBACK= int('000010',2)

  #Event type:
  EVENT_ENTRY= 0
  EVENT_STAY= 1
  EVENT_EXIT= 2
  EVENT_TIMER= 10

  #Return kind:
  COMPLETED= 0
  CONTINUE= 1
  EXCEPTION= 10

  #Initialize the assessment object.
  #callback_type: A combination of *_CALLBACK (e.g. STATE_CALLBACK | TIMER_CALLBACK)
  #time_step: Valid only when callback_type|TIMER_CALLBACK
  def __init__(self, callback_type, name='assessment', time_step=0.1):
    self.CallbackType= callback_type
    self.Name= name
    self.TimeStep= time_step

    #Function to execute assessment.
    #Return value should be:
    #  COMPLETED: remove the assessment model.
    #  CONTINUE: keep the assessment model.
    #  EXCEPTION: raise an exception (TEST).
    #bool Assess(assessment, event_type, context, sm)
    #  assessment: this object (self).
    #  event_type: one of TAssessment.EVENT_*
    #  context: list of current states.
    #    Note: Hierarchal state machine is possible.
    #    In order to distinguish a motion script (e.g. 'pour') and
    #    a state machine inside the script, we add 'sm' to context
    #    when we are in the state machine.
    #  sm: current state machine (STATE_CALLBACK) or None (TIMER_CALLBACK).
    self.Assess= None

    #List of states when this assessment model is registered.
    self.RegisteredContext= []

  def __repr__(self):
    return "{'Name':%r, 'Assess':%r, 'RegisteredContext':%r}" % (
        self.Name, self.Assess, self.RegisteredContext)



#Modify score from examples.
#score: Current score.
#good_exs, bad_exs: Examples from database.
#dist_func: Distance function between the current parameters/situation and an example.
#dist_threshold: Threshould of example; if the distance is less than this value, the score becomes None.
def ModifyScoreFromExamples(score, good_exs, bad_exs, dist_func, dist_threshold):
  assert(len(good_exs)==0) #FIXME: Not implemented for good examples

  if score is None:  return None

  for situation, inferred_data, assessment in bad_exs:
    #inferred_data[0][0] should be ['b53', 'l_x_grab']
    dist= dist_func(situation, inferred_data, assessment)
    if dist is None:  continue

    #print '##dist:',dist,dist>=dist_threshold
    if dist<dist_threshold:  return None
  return score


#Execute optimization.
#fmin_obj: objective function to be minimized.
#parameters0, scale0: default parameter mean and std-dev.
#options: options for CMA-ES and this function; 'bounds' must be defined.
#  options specific for this function are:
#    scale0_ratio1: ratio to modify scale0 when a value is found in the database.
#    scale0_ratio2: ratio to modify scale0 when a value is guessed randomly.
#    init_guess_num: number of values guessed randomly.
#    max_init_guess_num: maximum number of initial guess.
#             This is a list of two elements. The second value is used
#             only when no solution is found in initial guess.
#    db_search_num: number of values searched from database.
#    max_db_search_count: maximum number of database search.
#    db_param_novelty: only parameters whose distances are greater than this value are considered (i.e. ignoring similar examples in db).
#database, db_search_key: database and key to search in database.
#infer_type: inference type should be this.
def ExecInference(fmin_obj, parameters0, scale0, options,
                  database, db_search_key, infer_type):
  def pop_or(d, key, v_or):
    if key in d:
      v= d[key]
      del d[key]
      return v
    else:
      return v_or
  scale0_ratio1= pop_or(options,'scale0_ratio1',0.005)
  scale0_ratio2= pop_or(options,'scale0_ratio2',0.2)
  init_guess_num= pop_or(options,'init_guess_num',10)
  max_init_guess_num= pop_or(options,'max_init_guess_num',[1000,1000000])
  db_search_num= pop_or(options,'db_search_num',10)
  max_db_search_count= pop_or(options,'max_db_search_count',1000000)
  db_param_novelty= pop_or(options,'db_param_novelty',0.1)

  #Search from database:
  CPrint(1,'Searching from database...')
  fp_dat= []
  db_search_count= 0
  flag_to_end= False
  #Searching database from newer data:
  for db_situation, inferred_data, assessment in reversed(database):
    for keys, value in inferred_data:
      if (db_search_key in keys and
          'infer_info' in db_situation and
          'param' in db_situation['infer_info'] and
          'type' in db_situation['infer_info'] and
          infer_type==db_situation['infer_info']['type'] ):
        p= db_situation['infer_info']['param']
        #Check the novelty of the parameter p:
        p_novelties= [Dist(p,p1) for (f1,p1) in fp_dat]
        if len(p_novelties)==0 or min(p_novelties)>db_param_novelty:
          f= fmin_obj(p)
          if f is not None:
            fp_dat.append([f,p])
            if len(fp_dat)>=db_search_num:
              flag_to_end= True
              break
          db_search_count+= 1
          if db_search_count>=max_db_search_count:
            flag_to_end= True
            break
    if flag_to_end:
      break
  if len(fp_dat)>0:
    fp_dat.sort()
    parameters0= fp_dat[0][1]
    scale0= scale0_ratio1*scale0
    CPrint(1,'Found in database (%i)'%len(fp_dat))
  else:
    CPrint(0,'Not found in database')

  #Initial guess:
  if fmin_obj(parameters0) is None:
    CPrint(1,'Doing initial guess...')
    fp_dat= InitialGuess(options['bounds'], fmin_obj, num=init_guess_num, max_count=max_init_guess_num[0])
    if len(fp_dat)==0:
      #Retry:
      fp_dat= InitialGuess(options['bounds'], fmin_obj, num=1, max_count=(max_init_guess_num[1]-max_init_guess_num[0]))
    if len(fp_dat)>0:
      fp_dat.sort()
      parameters0= fp_dat[0][1]
      scale0= scale0_ratio2*scale0
      CPrint(1,'Success to initial guess (%i)'%len(fp_dat))
    else:
      CPrint(0,'Failed to initial guess')

  score0= fmin_obj(parameters0)
  maxfevals= options['maxfevals'] if 'maxfevals' in options else 1000000
  CPrint(1,'parameters0=',parameters0)
  CPrint(1,'      score=',-score0 if score0 is not None else None)
  CPrint(1,'  maxfevals=',maxfevals)

  #res= cma.fmin(fmin_obj, parameters0, scale0, options)
  es= cma.CMAEvolutionStrategy(parameters0, scale0, options)
  has_solution= False
  if score0 is not None:
    has_solution= True
    solutions= [parameters0]
    scores= [score0]
  else:
    solutions= []
    scores= []
  while not es.stop() and maxfevals>0:
    x= es.ask(1)[0]
    f= fmin_obj(x)
    maxfevals-= 1
    if f is not None:
      has_solution= True
      solutions.append(x)
      scores.append(f)
      if len(scores)>=es.popsize:
        es.tell(solutions, scores)
        es.disp()
        solutions= []
        scores= []
        #TEST
        #if es.result()[1]<>np.inf and -es.result()[1]>0.8:  break
  res= es.result()

  if res[0] is not None and res[1]<>np.inf:
    parameters_res= res[0]
    score_res= res[1]
  elif has_solution and len(solutions)>0:
    score_res,parameters_res= min(zip(scores,solutions))
  else:
    score_res= None
    parameters_res= None
  CPrint(1,'CMA-ES solution=',parameters_res)
  CPrint(1,'          score=',-score_res if score_res is not None else None)
  CPrint(1,'         fevals=',res[3])
  if has_solution:
    return parameters_res, score_res
  else:
    return None, None



#Return a Boltzmann policy (probabilities of selecting each action)
def BoltzmannPolicy(tau, values):
  if len(values)==0: return []
  max_v= max(values)
  sum_v= 0.0
  for q in values:
    sum_v+= math.exp((q-max_v)/tau)
  if sum_v<1.0e-10:
    return [1.0/float(len(values))]*len(values)
  probs= [0.0]*len(values)
  for d in range(len(values)):
    probs[d]= math.exp((values[d]-max_v)/tau)/sum_v
  return probs

#Return an action selected w.r.t. the policy (probabilities of selecting each action)
def SelectFromPolicy(probs):
  p= random.random()  #Random number in [0,1]
  action= 0
  for prob in probs:
    if p<=prob:  return action
    p-= prob
    action+= 1
  return action-1

#Learning to select a vector from discrete set
class TDiscParam:
  def __init__(self):
    #Parameter candidates which is a set of parameter vectors.
    self.Candidates= []
    self.Means= []
    self.SqMeans= []
    #Index of the parameter vector lastly selected.
    self.index= -1
    #Temparature parameter for the Boltzmann selection.
    self.BoltzmannTau= 0.1
    self.UCBNsd= 1.0
    self.Alpha= 0.2
    self.InitStdDev= 1.0
  def UCB(self):
    return [self.Means[d] + self.UCBNsd*math.sqrt(max(0.0,self.SqMeans[d]-self.Means[d]**2)) for d in range(len(self.Candidates))]
  #Initialize learner.  data: set to continue from previous state, generated by Save.
  def Init(self, data=None):
    self.index= -1
    if data is not None:
      self.Candidates= data['candidates']
      self.Means= data['means']
      self.SqMeans= data['sqmeans']
      self.BoltzmannTau= data['boltzmann_tau']
      self.UCBNsd= data['ucb_nsd']
      self.Alpha= data['alpha']
      self.InitStdDev= data['init_std_dev']
  #Returns the latest selected parameter.
  def Param(self):
    if len(self.Candidates)>0 and self.index>=0:
      return self.Candidates[self.index]
    return None
  def Select(self):
    if len(self.Candidates)>0:
      if len(self.Candidates)!=len(self.Means):
        self.Means= [0.0]*len(self.Candidates)
        self.SqMeans= [self.InitStdDev**2]*len(self.Candidates)
        #print 'Warning: Means is not initialized.  Using %r.' % (self.Means)
      ucb= self.UCB()
      probs= BoltzmannPolicy(self.BoltzmannTau,ucb)
      self.index= SelectFromPolicy(probs)
      CPrint(1,'TDiscParam:DEBUG: Param:%r Index:%i UCB:%f' % (self.Candidates[self.index],self.index,ucb[self.index]))
    else:
      self.index= -1
  def Update(self,score):
    if self.index>=0:
      self.Means[self.index]= self.Alpha*score + (1.0-self.Alpha)*self.Means[self.index]
      self.SqMeans[self.index]= self.Alpha*(score**2) + (1.0-self.Alpha)*self.SqMeans[self.index]
      CPrint(1,'TDiscParam:DEBUG: Index:%i Score:%f New-Mean:%f' % (self.index,score,self.Means[self.index]))
  #Save internal parameters as a dictionary.
  def Save(self):
    data= {}
    data['candidates']= ToStdType(self.Candidates)
    data['means']= ToStdType(self.Means)
    data['sqmeans']= ToStdType(self.SqMeans)
    data['boltzmann_tau']= self.BoltzmannTau
    data['ucb_nsd']= self.UCBNsd
    data['alpha']= self.Alpha
    data['init_std_dev']= self.InitStdDev
    return copy.deepcopy(data)

#Learning to a continuous value vector whose gradient is known
class TContParamGrad:
  def __init__(self):
    self.Mean= []
    self.Min= []
    self.Max= []
    #Function to compute a gradient from argv (parameter of Update)
    self.Gradient= None
    self.Alpha= 0.2
  #Initialize learner.  data: set to continue from previous state, generated by Save.
  def Init(self, data=None):
    if data is not None:
      self.Mean= data['mean']
      self.Min= data['min']
      self.Max= data['max']
      self.Alpha= data['alpha']
  #Returns the latest selected parameter.
  def Param(self):
    return self.Mean
  def Select(self):
    CPrint(1,'TContParamGrad:DEBUG: Param:%r' % (self.Mean))
  def Update(self,argv):
    if not self.Gradient:
      CPrint(1,'TContParamGrad:Error: No gradient function')
      return
    gradient= self.Gradient(argv)
    assert len(gradient)==len(self.Mean)
    self.Mean= [self.Mean[d] + self.Alpha*gradient[d] for d in range(len(gradient))]
    if len(self.Min)>0:
      self.Mean= [max(self.Mean[d],self.Min[d]) for d in range(len(gradient))]
    if len(self.Max)>0:
      self.Mean= [min(self.Mean[d],self.Max[d]) for d in range(len(gradient))]
    CPrint(1,'TContParamGrad:DEBUG: Grad:%r New-Mean:%r' % (gradient,self.Mean))
  #Save internal parameters as a dictionary.
  def Save(self):
    data= {}
    data['mean']= self.Mean
    data['min']= self.Min
    data['max']= self.Max
    data['alpha']= self.Alpha
    return copy.deepcopy(data)

#Learning to a continuous value vector whose gradient is unknown
class TContParamNoGrad:
  def __init__(self):
    self.Mean= []
    self.Std= 1.0
    self.Min= []
    self.Max= []
    self.CMAESOptions= {}
    t= time.localtime()
    self.Logger= '%s/data/tmp/cmaes%02i%02i%02i%02i%02i%02i.dat' % (os.environ['HOME'],t.tm_year%100,t.tm_mon,t.tm_mday,t.tm_hour,t.tm_min,t.tm_sec)
    self.Generation= 0
  #Initialize learner.  data: set to continue from previous state, generated by Save.
  def Init(self, data=None):
    if len(self.Min)>0 or len(self.Max)>0:
      self.CMAESOptions['bounds']= [self.Min,self.Max]
    if data is not None:
      InsertDict(self.CMAESOptions, data['options'])
      self.Mean= data['xmean']
      self.Std= 1.0
      self.CMAESOptions['scaling_of_variables']= data['stds']
    self.es= cma.CMAEvolutionStrategy(self.Mean, self.Std, self.CMAESOptions)
    self.solutions= []
    self.scores= []
    if data is not None:
      self.solutions= data['solutions']
      self.scores= data['scores']
      self.Logger= data['logger']
      self.tmpfp= file(self.Logger,'a')
      self.Generation= data['generation']
    else:
      self.tmpfp= file(self.Logger,'w')
  #Returns the latest selected parameter.
  def Param(self):
    if len(self.solutions)>0:
      return self.solutions[-1]
    return None
  def Select(self):
    #For the previous selection was not evaluated...
    while len(self.scores)<len(self.solutions):
      self.solutions.pop()
    self.solutions.append(self.es.ask(1)[0])
    CPrint(1,'TContParamNoGrad:DEBUG: Param:%r' % (self.solutions[-1]))
  def Update(self,score):
    if len(self.scores)==len(self.solutions)-1:
      self.scores.append(-score)
    if len(self.solutions)==self.es.popsize:
      self.es.tell(self.solutions, self.scores)
      self.es.disp()
      for i in range(len(self.solutions)):
        self.tmpfp.write('%i %s %f\n'%(self.Generation,' '.join(map(str,self.solutions[i])),self.scores[i]))
      self.tmpfp.write('\n\n')
      self.solutions= []
      self.scores= []
      self.es.disp()
      self.Generation+= 1
    CPrint(1,'TContParamNoGrad:DEBUG: Score:%f' % (score,))
  #Save internal parameters as a dictionary.
  def Save(self):
    data= {}
    data['options']= ToStdType(self.CMAESOptions)
    data['xmean']= ToStdType(self.es.result()[5])
    data['stds']= ToStdType(self.es.result()[6])
    data['solutions']= ToStdType(self.solutions)
    data['scores']= ToStdType(self.scores)
    while len(data['scores'])<len(data['solutions']):
      data['solutions'].pop()
    data['logger']= self.Logger
    data['generation']= self.Generation
    return copy.deepcopy(data)

